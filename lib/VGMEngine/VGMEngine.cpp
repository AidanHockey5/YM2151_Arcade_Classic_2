#include "VGMEngine.h"

VGMEngineClass::VGMEngineClass()
{
    MegaStream_Create(&stream, buf, VGM_BUF_SIZE);
    MegaStream_Create(&wavStream, wavBuf, WAV_BUF_SIZE);
}
VGMEngineClass::~VGMEngineClass(){}

void VGMEngineClass::ramtest() //Tests 256 bytes of external RAM
{
    #if ENABLE_SPIRAM
        const uint32_t ram_test_size = 0xFF;
        bool pass = false;
        ram.Init();
        for(uint32_t i = 0; i<ram_test_size; i++)
            ram.WriteByte(i, i);
        for(uint32_t i = 0; i<ram_test_size; i++)
        {
            Serial.println(ram.ReadByte(i));
            if(ram.ReadByte(i) != i)
                pass = false;
            else
                pass = true;
        }
        pass == true ? Serial.println("RAM CHECK PASS") : Serial.println("RAM CHECK FAIL!!!");
    #endif
}

bool VGMEngineClass::begin(File *f)
{
    ready = false;
    file = f;
    if(!header.read(file))
    {
        state = ERROR;
        return false;
    }
    gd3.read(file, header.gd3Offset+0x14);
    
    if(header.vgmDataOffset == 0)
        file->seek(0x40);
    else
        file->seek(header.vgmDataOffset+0x34);
    
    if(header.gd3Offset != 0)
    {
        loopPos = header.gd3Offset+0x14-1;
    }
    else
    {
        loopPos = header.EoF+4-1;
    }

    isOneOff = header.loopOffset == 0;
    //stopDacStreamTimer();
    chipSetup();
    #if ENABLE_SPIRAM
        ram.Init();
    #endif
    //resetDataBlocks();
    dacSampleReady = false;
    activeDacStreamBlock = 0xFF;
    storePCM();
    pcmBufferPosition = 0;
    waitSamples = 0;
    loopCount = 0;
    badCommandCount = 0;
    dacSampleCountDown = 0;

    wavStartOffset = 0;
    wavPos = 0;
    wavLoopPos = 0;
    wavEnabled = false;
    analogWrite(A0, 0); //Reset the DACs
    analogWrite(A1, 0);

    if(header.indent == 0x20706756) //Vgp file detected. There is WAV data at the bottom of this file!
    {
        uint32_t prevPos = file->position(); //Keep track of the file pointer so we can put it back after we check out the WAV metadata
        file->seekEnd(-4); //WAV start offset is stored at the very end of the file
        file->read(&wavStartOffset, 4); //It is a 32-bit file pointer
        wavStartOffset += 0x44; //Skip WAV header and go right to data
        wavPos = wavStartOffset; //Initialize the WAV file pointer to the start of the data
        wavLoopPos = wavStartOffset + (header.totalSamples*4);  //The wavLoopPos is the location in the WAV data where the buffer should jump back in the file to the main portion of the current track
                                                                //*4 because 16 bits per sample, aka 2 bytes per each channel's sample, and this is a stereo stream, so multiply again by 2
        wavEnabled = true;
        file->seekSet(prevPos); //Put that file pointer back where it came from or so help me
    }

    MegaStream_Reset(&stream);
    MegaStream_Reset(&wavStream);
    load();
    if(wavEnabled)
    {
        loadWav();
    }
    state = PLAYING;
    ready = true;
    return true;
}

// void VGMEngineClass::resetDataBlocks()
// {
//     memset(dataBlocks, 0, sizeof(dataBlocks));
// }

uint8_t empty = 0;
uint8_t VGMEngineClass::readBufOne(MegaStreamContext_t *s)
{
    if(MegaStream_Used(s) < 1)
    {
        //digitalWrite(PA8, HIGH);
        s == &stream ? load() : loadWav(); //I hate this line. Fix later... or not. Idk, you're not my mom. Which stream should I fill, the VGM stream, or the WAV stream?
        empty = 1;
    }
    uint8_t b[1];
    MegaStream_Recv(s, b, 1);
    return b[0];
}

uint16_t VGMEngineClass::readBuf16(MegaStreamContext_t *s)
{
    if(MegaStream_Used(s) < 2)
    {
        //digitalWrite(PA8, HIGH);
        s == &stream ? load() : loadWav();
        empty = 16;
    }
    uint16_t d;
    uint8_t b[2];
    MegaStream_Recv(s, b, 2);
    d = uint16_t(b[0] + (b[1] << 8));
    return d;
}

uint32_t VGMEngineClass::readBuf32(MegaStreamContext_t *s)
{
    if(MegaStream_Used(s) < 4)
    {
        //digitalWrite(PA8, HIGH);
        s == &stream ? load() : loadWav();
        empty = 32;
    }
    uint32_t d;
    uint8_t b[4];
    MegaStream_Recv(s, b, 4);
    d = uint32_t(b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24));
    return d;
}

bool VGMEngineClass::topUp()
{
    if(MegaStream_Free(&stream) == 0)
        return true;
    byte b = file->read();
    MegaStream_Send(&stream, &b, 1);
    return false;
}

bool VGMEngineClass::storePCM(bool skip) //This function effectively seeks through the PCM data in the VGM file and DOES NOT STORE IT as we will be emulating any PCM sounds through WAV data
{
    bool isPCM = false;
    uint16_t count = 0;
    pcmBufferEndPosition = 0;
    while(file->peek() == 0x67)
    {
        if(skip)
        {
            file->seek(pcmSkipPosition);
            return true;
        }
        isPCM = true;
        uint32_t size = 0;
        file->read(); //0x67
        file->read(); //0x66
        file->read(); //datatype
        file->read(&size, 4); //PCM chunk size
        pcmBufferEndPosition += size;
        //file->seek(pcmBufferEndPosition);
        count++;
        if(header.vgmDataOffset == 0)
        {
            pcmSkipPosition = 0x40+pcmBufferEndPosition+(7*count); //7 is for the PCM block header info and datasize and needs to be multiplied by the datablock # (count)
            file->seek(pcmSkipPosition); 
        }
        else
        {
            pcmSkipPosition = header.vgmDataOffset+0x34+pcmBufferEndPosition+(7*count);
            file->seek(pcmSkipPosition);
        }
    }
    return isPCM;
}

bool VGMEngineClass::load(bool singleChunk)
{
    const uint16_t MAX_CHUNK_SIZE = 512;
    int32_t space = MegaStream_Free(&stream);
    if(space == 0)
        return true;
    bool didSingleChunk = false;
    uint8_t chunk[MAX_CHUNK_SIZE];
    while(MegaStream_Free(&stream) != 0 || didSingleChunk) //Fill up the entire buffer, or only grab a single chunk quickly
    {
        bool hitLoop = false;
        uint16_t chunkSize = min(space, MAX_CHUNK_SIZE); //Who's smaller, the space left in the buffer or the maximum chunk size?
        
        if(file->position() + chunkSize >= loopPos+1) //Loop code. A bit of math to see where the file pointer is. If it goes over the 0x66 position, we'll set a flag to move the file pointer to the loop point and adjust the chunk as to not grab data past the 0x66
        {
            chunkSize = loopPos+1 - file->position(); //+1 on loopPos is to make sure we include the 0x66 command in the buffer in order for loop-triggered events to work.
            hitLoop = true;
        }

        space -= chunkSize;                 //Reduce space by the chunkSize, then read from the SD card into the chunk. Send chunk to buffer.
        file->read(chunk, chunkSize);
        MegaStream_Send(&stream, chunk, chunkSize); 
        if(hitLoop)                         //Here is where we reset the file pointer back to the loop point
        {
            if(header.loopOffset !=0)
                file->seek(header.loopOffset+0x1C);
            else
            {
                if(header.vgmDataOffset == 0)
                    file->seek(0x40);
                else
                    file->seek(header.vgmDataOffset+0x34);
                storePCM(true);
            }
        }
        if(space <= 0)                      //No more space in the buffer? Just eject.
            return true;
        if(singleChunk)                     //Only want to grab a single chunk instead of filling the entire buffer? Set this flag.
            didSingleChunk = true;
    }
    //NOTE, Loop (0x66) will ALWAYS be (Gd3 offset - 1) OR (EoF offset - 1) if there is no Gd3 data
    return false;
}

bool VGMEngineClass::loadWav(bool singleChunk)
{
    const uint16_t MAX_CHUNK_SIZE = 512;
    int32_t space = MegaStream_Free(&wavStream);
    if(space == 0)
        return true;
    uint32_t lastPosInFile = file->position(); //Since we're stealing the file pointer away from the main VGM buffer, we need to record it so we can put it back after our operation
    file->seekSet(wavPos); //Go back to where we were last reading WAV data in the file
    bool didSingleChunk = false;
    uint8_t chunk[MAX_CHUNK_SIZE];
    while(MegaStream_Free(&wavStream) != 0 || didSingleChunk) //Fill up the entire buffer, or only grab a single chunk quickly
    {
        bool hitLoop = false;
        uint16_t chunkSize = min(space, MAX_CHUNK_SIZE); //Who's smaller, the space left in the buffer or the maximum chunk size?
        
        if(file->position() + chunkSize >= wavLoopPos) //Detect if we're past the WAV loop point
        {
            chunkSize = wavLoopPos - file->position(); //Only take x amount of bytes instead of a full chunk so we don't grab garbage data
            hitLoop = true;
            Serial.print("HIT LOOP AT: 0x"); Serial.println(file->position(), HEX);
            Serial.print("Chunk Size: "); Serial.println(chunkSize);
        }

        space -= chunkSize;                 //Reduce space by the chunkSize, then read from the SD card into the chunk. Send chunk to buffer.
        file->read(chunk, chunkSize);
        wavPos += chunkSize; //Keep track of where we're reading WAV bytes for later
        MegaStream_Send(&wavStream, chunk, chunkSize); 
        if(hitLoop)                         //Here is where we reset the file pointer back to the loop point
        {
            uint32_t wavLoopStartPos;
            if(header.loopOffset == 0) //No loop in this file, so just go back to the start of the wav data
                wavLoopStartPos = wavStartOffset;
            else //Otherwise, calculate where the start of the loop is in the WAV data *past* the introduction portion of the track
                wavLoopStartPos = wavStartOffset + ((header.totalSamples - header.loopNumSamples)*4); //*4 because 16 bits per sample, aka 2 bytes per each channel's sample, and this is a stereo stream, so multiply again by 2. Skip the intro portion of the track.
            
            file->seekSet(wavLoopStartPos);
            wavPos = wavLoopStartPos; //Keep track of the WAV pointer for next time
        }
        if(space <= 0)                      //No more space in the buffer? Just eject.
        {
            file->seekSet(lastPosInFile); //Remember to put the file pointer back so you don't confuse the VGM buffer!
            return true;
        }
        if(singleChunk)                     //Only want to grab a single chunk instead of filling the entire buffer? Set this flag.
            didSingleChunk = true;
    }
    file->seekSet(lastPosInFile);
    return false;
}

void VGMEngineClass::chipSetup()
{
    si5351.reset();
    delay(10);
    #if ENABLE_SN76489
    sn76489->setClock(header.sn76489Clock);
    si5351.set_freq(header.sn76489Clock*SI5351_FREQ_MULT, SI5351_CLK1);
    delay(10);
    sn76489->reset();
    #endif
    #if ENABLE_YM2612
    ym2612->setClock(header.ym2612Clock);
    si5351.set_freq(header.ym2612Clock*SI5351_FREQ_MULT, SI5351_CLK0); //CLK0 YM
    delay(10);
    ym2612->reset();
    #endif
    #if ENABLE_YM2151
    ym2151->setClock(header.ym2151Clock);
    si5351.set_freq(header.ym2151Clock*SI5351_FREQ_MULT, SI5351_CLK0); //CLK0 YM
    delay(10);
    ym2151->reset();
    #endif
}

void VGMEngineClass::tick44k1()
{
    if(!ready)
        return;
    waitSamples--;     
    if(wavEnabled)
        playWavSample();
}

void VGMEngineClass::tickDacStream()
{
    if(!ready)
        return;
    if(activeDacStreamBlock != 0xFF)
        dacSampleCountDown--;
}

void VGMEngineClass::playWavSample()
{
    int16_t l = readBuf16(&wavStream);
    int16_t r = readBuf16(&wavStream);

    //16 bit signed to 12 bit unsigned conversion
    REG_DAC_DATA0 = ((l + 32768) >> 4); //Quickly write to the ADC A0   
    REG_DAC_DATA1 = ((r + 32768) >> 4); //Quickly write to the ADC A1 
}

VGMEngineState VGMEngineClass::play()
{
    switch(state)
    {
    case IDLE:
        return IDLE;
    break;
    case END_OF_TRACK:
        state = IDLE;
        return END_OF_TRACK;
    break;
    case PLAYING:
        load(); 
        if(wavEnabled)
            loadWav();
        #if ENABLE_YM2612
        while(dacSampleCountDown <= 0)
        {
            dacSampleReady = false;
            if(dacStreamBufPos+1 < dataBlocks[activeDacStreamBlock].DataStart+dataBlocks[activeDacStreamBlock].DataLength)
            {
                uint8_t data = ram.ReadByte(dacStreamBufPos++);
                // check if channel 6 is enabled, since this is a DAC write
                if (ym2612CHControl[0x06])
                {
                    ym2612->write(0x2A, data, 0);
                }
            }
            else if(bitRead(dataBlocks[activeDacStreamBlock].LengthMode, 7)) //Looping length mode defined in 0x93 DAC STREAM
            {
                dacStreamBufPos = dataBlocks[activeDacStreamBlock].DataStart;
            }
            else
                activeDacStreamBlock = 0xFF;
            dacSampleCountDown++;
        }
        #endif
        while(waitSamples <= 0)
        {
            isBusy = true;
            waitSamples += parseVGM();
        }

        isBusy = false;
        if(loopCount == maxLoops)
        {
            state = END_OF_TRACK;
        }
        if(isOneOff)
        {
            if(loopCount == 1 && !loopOneOffs)
                state = END_OF_TRACK;
        }
        return PLAYING;
    break;
    case ERROR:
        return ERROR;
    break;
    }
}

uint16_t VGMEngineClass::getLoops()
{
    return loopCount;
}

uint16_t VGMEngineClass::parseVGM()
{
    uint8_t cmd;
    while(true)
    {
        cmd = readBufOne(&stream);
        switch(cmd)
        {
            case 0x4F:
            case 0x50:
            {
                #if ENABLE_SN76489
                // check for game gear stereo write
                if (cmd == 0x4F)
                {
                    sn76489->write(0x06);
                }
                uint8_t data = readBufOne();
                // check for a latch write (MSB=1)
                if ((data & 0x80) != 0)
                {
                    // decode the channel and store it
                    sn76489Latched = (data & 0x60) >> 5;
                    // if the latched channel is disabled then make it silent
                    if (!sn76489CHControl[sn76489Latched])
                    {
                        // override the write to volume full attenuation
                        data = (data & 0xE0) + 0x1F;
                    }
                    sn76489->write(data);
                }
                else
                {
                    // only write data if the latched channel is enabled
                    if (sn76489CHControl[sn76489Latched])
                    {
                        sn76489->write(data);
                    }
                }
                #endif
                break;
            }
            case 0x52:
            {
                #if ENABLE_YM2612
                uint8_t addr = readBufOne();
                uint8_t data = readBufOne();
                // check for an operator/channel key on/off write
                if (addr == 0x28)
                {
                    uint8_t channel = (data & 0x07);
                    // if the channel is disabled then make it silent (force a key-off)
                    if (!ym2612CHControl[channel])
                    {
                        // override the write disabling the operators
                        data = (data & 0x0F);
                    }
                }
                ym2612->write(addr, data, 0);
                #endif
                break;
            }
            case 0x53:
                // part 2 writes don't deal with operator key on/off operations
                #if ENABLE_YM2612
                ym2612->write(readBufOne(), readBufOne(), 1);
                #endif
            break;
            case 0x54:
            {
                #if ENABLE_YM2151
                uint8_t addr = readBufOne(&stream);
                uint8_t data = readBufOne(&stream);
                ym2151->write(addr, data);
                #endif
            }
            break;
            case 0x61:
                return readBuf16(&stream);
            case 0x62:
                return 735;
            case 0x63:
                return 882;
            case 0x67: //Skip PCM data block
            {
                readBufOne(&stream);
                readBufOne(&stream);
                readBufOne(&stream);
                uint32_t pcmSize = readBuf32(&stream); //Payload size;
                for(uint32_t i=0; i<pcmSize; i++)
                    readBufOne(&stream);                
            }
            break;
            case 0xA0:
            case 0xB0:
            case 0xB1:
            case 0xB2:
            case 0xB5: //Ignore common secondary PCM chips
            case 0xB6:
            case 0xB7:
            case 0xB8:
            case 0xB9:
            case 0xBA:
            case 0xBB:
            case 0xBC:
            case 0xBD:
            case 0xBE:
            case 0xBF:
                readBuf16(&stream);
            break;
            case 0xC0: //24 bit write PCM chips
            case 0xC1:
            case 0xC2:
            case 0xC3:
            case 0xC4:
            case 0xC5:
            case 0xC6:
            case 0xC7:
            case 0xC8:
            case 0xD0:
            case 0xD1:
            case 0xD2:
            case 0xD3:
            case 0xD4:
            case 0xD5:
            case 0xD6:
                readBufOne(&stream); readBufOne(&stream); readBufOne(&stream);
            break;
            case 0xE1: //32 bit write PCM chips
                readBuf32(&stream);
            break;
            case 0x70:
            case 0x71:
            case 0x72:
            case 0x73:
            case 0x74:
            case 0x75:
            case 0x76:
            case 0x77:
            case 0x78:
            case 0x79:
            case 0x7A:
            case 0x7B:
            case 0x7C:
            case 0x7D:
            case 0x7E:
            case 0x7F:
                return (cmd & 0x0F)+1;//+1; Removed +1. Remember, even if you return a 0, there is always going to a latency of at least 1 tick
            case 0x80:
            case 0x81:
            case 0x82:
            case 0x83:
            case 0x84:
            case 0x85:
            case 0x86:
            case 0x87:
            case 0x88:
            case 0x89:
            case 0x8A:
            case 0x8B:
            case 0x8C:
            case 0x8D:
            case 0x8E:
            case 0x8F:
            {
                #if ENABLE_YM2612
                uint8_t data = ram.ReadByte(pcmBufferPosition++);
                // check if channel 6 is enabled, since this is a DAC write
                if (ym2612CHControl[0x06])
                {
                    ym2612->write(0x2A, data, 0);
                }
                uint8_t wait = (cmd & 0x0F);
                if(wait == 0)
                    break;
                else
                    return wait; //Wait -1: Even if you return a 0, every loop has inherent latency of at least 1 tick. This compensates for that.
                #endif
            }
            break;
            case 0xE0:
                pcmBufferPosition = readBuf32(&stream);
            break;
            case 0x66:
            {
                //Loop
                if(maxLoops != 0xFFFF) //If sent to short int max, just loop forever
                    loopCount++;
                activeDacStreamBlock = 0xFF;
                return 0;
            }
            case 0x90:
            {
                readBuf16(&stream);//skip stream ID and chip type
                dataBlockChipPort = readBufOne(&stream);
                dataBlockChipCommand = readBufOne(&stream);
                //Serial.print("0x90: ");// Serial.print(streamID, HEX); Serial.print(" "); Serial.print(chiptype, HEX); Serial.print(" "); Serial.print(pp, HEX); Serial.print(" "); Serial.print(cc, HEX); Serial.println("   --- SETUP STREAM CONTROL");
            }
            break;
            case 0x91:
            {
                readBuf16(&stream); //Skip stream ID and Data bank ID. Stream ID will be const and so will data bank if you're just using OPN2/PSG
                dataBlockStepSize = readBufOne(&stream);
                dataBlockStepBase = readBufOne(&stream);
                //Serial.println("0x91: "); //Serial.print(streamID, HEX); Serial.print(" "); Serial.print(dbID, HEX); Serial.print(" "); Serial.print(ll, HEX); Serial.print(" "); Serial.print(bb, HEX); Serial.println("   --- SET STREAM DATA");
            }
            break;
            case 0x92:
            {
                readBufOne(&stream); //skip stream ID
                uint32_t streamFrq = readBuf32(&stream);
                //setDacStreamTimer(streamFrq);
                //Serial.println("0x92: "); //Serial.print(streamID, HEX); Serial.print(" "); Serial.print(streamFrq); Serial.println("   --- SET STREAM FRQ");
            }
            break;
            case 0x93:
            {
                //Come back to this one
                readBufOne(&stream); //skip stream ID
                dacStreamBufPos = readBuf32(&stream);
                uint8_t lmode = readBufOne(&stream); //Length mode
                //Serial.print("LMODE: "); Serial.print(lmode, BIN);
                dacStreamCurLength = readBuf32(&stream);
                // //Serial.print("BUF POS: "); Serial.println(dacStreamBufPos);
                // for(uint8_t i = 0; i<MAX_DATA_BLOCKS_IN_BANK; i++)
                // {
                //     if(dataBlocks[i].absoluteDataStartInBank == dacStreamBufPos)
                //     {
                //         activeDacStreamBlock = i;
                //         dataBlocks[i].LengthMode = lmode;
                //         //Serial.print("FOUND BLOCK: "); Serial.println(i);
                //         break;
                //     }
                // }
                //Serial.println("0x93: "); //Serial.print(streamID, HEX); Serial.print(" "); Serial.print(dStart, HEX); Serial.print(" "); Serial.print(lengthMode, HEX); Serial.print(" "); Serial.print(dLength); Serial.println("   --- START STREAM");
            }
            break;
            case 0x94:
            {
                readBufOne(&stream); //skip stream ID
                // stopDacStreamTimer();
                // activeDacStreamBlock = 0xFF;
                //Serial.println("0x94: "); //Serial.print(streamID, HEX); Serial.print(" "); Serial.println("   --- STOP STREAM");
            }
            break;
            case 0x95:
            {
                readBufOne(&stream); //skip stream ID
                uint16_t blockID = readBuf16(&stream);
                uint8_t flags = readBufOne(&stream); //flags
                // dacStreamBufPos = dataBlocks[blockID].DataStart;
                // dacStreamCurLength = dataBlocks[blockID].DataLength;
                // activeDacStreamBlock = blockID;
                // dataBlocks[blockID].LengthMode = bitRead(flags, 0) == 1 ? 0b10000001 : 0; //Set proper loop flag. VGMSpec, why the hell did you pick a different bit for the same flag as the 0x93 command. Stupid!
                //Serial.print("0x95: "); // Serial.print(streamID, HEX); Serial.print(" "); Serial.print(blockID, HEX); Serial.print(" "); Serial.print(flags, HEX); Serial.println("   --- START STREAM FAST");
                //Serial.print("BLOCK ID: "); Serial.println(blockID);
            }
            break;

            default:
                Serial.print("E:"); Serial.println(cmd, HEX);
                badCommandCount++;
                if(badCommandCount >= COMMAND_ERROR_SKIP_THRESHOLD)
                    state = END_OF_TRACK;
                return 0;
        }
    }    
}

VGMEngineClass VGMEngine;