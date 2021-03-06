// vgm_to_vgp.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "decompress.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <Windows.h>
#include <tchar.h>
#include <cstdlib>
#include <locale>
#include <codecvt>

std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

namespace fs = std::experimental::filesystem;
using namespace std;

bool ConvertFile(string path);
wstring ExePath();
wstring remove_extension(const wstring& filename);
string remove_extension_std(const string& filename);
wstring dirnameOf(const wstring& fname);
wstring ExtractFileName(const wstring& fullPath);


const string VGMPLAY_LOCATION = "\"./bin/vgm_play.exe\"";

vector<string> dirList;
vector<string> fileList;

int main(int argc, char *argv[])
{	
	int dirCount = 0;
	int fileCount = 0;
	for (int i = 1; i < argc; i++) //Ignore the first path, it is this program's own path
	{
		struct stat s;
		if (stat(argv[i], &s) == 0)
		{
			if (s.st_mode & S_IFDIR)
			{
				for (const auto & entry : fs::directory_iterator(argv[i]))
				{
					string p = entry.path().string();
					struct stat sub;
					if (stat(p.c_str(), &sub) == 0)
						if(sub.st_mode & S_IFREG)
							fileList.push_back(p);
				}
			}
			else if (s.st_mode & S_IFREG)
				fileList.push_back(argv[i]);
		}
	}

	for (int i = 0; i < fileList.size(); i++)
	{
		string f = fileList.at(i);
		ConvertFile(f);
		//cout << f << endl;
	}

	string a;
	cin >> a;
    return 0;
}

bool ConvertFile(string path)
{
	struct stat fileInfo;
	string decompPath; //Temporary path to a decompressed VGM file just in case the file is vgz
	bool succeed = false;
	
	char headerBuf[0xFF]; //Grab the first 0xFF bytes of the file. This will be to read-in mostly the vgm magic number data, but it might be useful later to know all of the header info
	ifstream vgmfile(path, ios::in | ios::binary);
	vgmfile.read(headerBuf, 0xFF);
	uint32_t magic = uint32_t(headerBuf[0] + (headerBuf[1] << 8) + (headerBuf[2] << 16) + (headerBuf[3] << 24));
	bool isVGZ = (magic << 16 == 0x8B1F0000);

	if (isVGZ) //If the file read has a gzip tag in its header, close the file, extract vgm file, re-open and read header again for new magic #
	{
		vgmfile.close();
		decompPath = remove_extension_std(path) + ".vgm";
		Decompress(path.c_str(), decompPath.c_str());
		vgmfile.open(decompPath, ios::in | ios::binary);
		vgmfile.read(headerBuf, 0xFF);
		magic = 0;
		magic = uint32_t(headerBuf[0] + (headerBuf[1] << 8) + (headerBuf[2] << 16) + (headerBuf[3] << 24));
	}

	if (magic == 0x206D6756) //Valid VGM File
	{
		cout << "Valid VGM!" << endl;
		wstring exepath = ExePath(); //Get relative path from this exe.

		//Create a WAV File from the VGM
		wstring arg;
		wstring widepath = isVGZ ? converter.from_bytes(decompPath) : converter.from_bytes(path);
		uint32_t fsize = 0;
		fsize = fileSize_wide(widepath.c_str()); //fileSize_wide defined in decompress.h
		cout << "File Size: " << fsize << endl;
		arg = exepath + L"/bin/VGMPlay.exe \"" + widepath + L"\"";
		_wsystem(arg.c_str());

		//Convert WAV to proper format (44.1KHz, 8-bit Stereo PCM)
		arg = exepath + L"/bin/sox/sox.exe ";
		wstring wavPath = L"\"" + dirnameOf(widepath) + L"\\" + remove_extension(ExtractFileName(widepath)) + L".wav\"";

		arg += wavPath + L" ";
		ifstream wavFile(wavPath, ios::in | ios::binary);
		char wavHeaderBuf[34];
		wavFile.read(wavHeaderBuf, 34);
		wavFile.close();
		uint32_t sampleRate = uint32_t(wavHeaderBuf[24] + (wavHeaderBuf[25] << 8) + (wavHeaderBuf[26] << 16) + (wavHeaderBuf[27] << 24)); //Sample rate starts at 0x18 (24)
		
		if (sampleRate != 44100) //Downsample or upsample to 44.1KHz
			arg += L"-r 44100 ";
		arg += L"-b 16 \""; //Change bitdepth to 16
		arg += dirnameOf(widepath) + L"/" + L"00000000.wav";
		
		//Full arg should be an expanded version of "sox in.wav -r 44100 -b 16 out.wav"
		//VGMPlay should already provide a stereo file, so no need to worry about that.
		cout << "Converting to 44.1KHz 16-bit Stereo PCM..." << endl;
		_wsystem(arg.c_str());

		wavPath.erase(std::remove(wavPath.begin(), wavPath.end(), '\"'), wavPath.end()); //Remove quotes from path, required for file delete.

		if (_wremove(wavPath.c_str()) == -1)
			wcout << L"Could not remove: " << wavPath.c_str() << endl;
		else
			wcout << L"Removed: " << wavPath << endl;

		//This section concatenates the PCM file to vgm file into new "vgp" file.
		vgmfile.clear();
		vgmfile.seekg(0); //Push vgmfile pointer back to the start position.

		//Configure wav file to point to newly processed temp wav file
		wavPath = dirnameOf(widepath) + L"/" + L"00000000.wav";
		wavFile = ifstream(wavPath, ios_base::binary);

		ofstream vgpFile(dirnameOf(widepath) + L"/" + remove_extension(ExtractFileName(widepath)) + L".vgp", ios_base::binary);
		
		vgpFile << vgmfile.rdbuf() << wavFile.rdbuf(); //Perform cat

		vgpFile.write(reinterpret_cast<const char *>(&fsize), sizeof(fsize)); //Add WAV offset location to end of file so player can seek to start of WAV data				   

		vgmfile.clear();
		vgmfile.seekg(0); //Push vgmfile pointer back to the start position.

		vgpFile.close();

		fstream vgpFileTrunc(dirnameOf(widepath) + L"/" + remove_extension(ExtractFileName(widepath)) + L".vgp", std::ios_base::binary | std::ios_base::out | std::ios_base::in);
		vgpFileTrunc.seekp(0, std::ios_base::beg);
		uint32_t vgpHeader = 0x20706756; //Replace "Vgm" header with "Vgp" to let player know to look for WAV offset 
		vgpFileTrunc.write(reinterpret_cast<const char *>(&vgpHeader), sizeof(vgpHeader));
		vgpFileTrunc.close();
		
		wavFile.close();

		if (_wremove(wavPath.c_str()) == -1)
			wcout << L"Could not remove: " << wavPath.c_str() << endl;
		else
			wcout << L"Removed: " << wavPath << endl;

		if (isVGZ)
		{
			vgmfile.close();
			_wremove(widepath.c_str());
		}

		cout << "DONE!" << endl;
		succeed = true;
	}
	else
	{
		cout << "INVALID VGM! BAD MAGIC" << endl;
		succeed = false;
	}
	vgmfile.close();
	return succeed;
}

wstring ExePath() 
{
	TCHAR buffer[MAX_PATH] = { 0 };
	GetModuleFileName(NULL, buffer, MAX_PATH);
	std::wstring::size_type pos = std::wstring(buffer).find_last_of(L"\\/");
	return std::wstring(buffer).substr(0, pos);
}

wstring remove_extension(const wstring& filename) 
{
	size_t lastdot = filename.find_last_of(L".");
	if (lastdot == wstring::npos) return filename;
	return filename.substr(0, lastdot);
}

string remove_extension_std(const string& filename)
{
	size_t lastdot = filename.find_last_of(".");
	if (lastdot == string::npos) return filename;
	return filename.substr(0, lastdot);
}

wstring dirnameOf(const wstring& fname)
{
	size_t pos = fname.find_last_of(L"\\/");
	return (wstring::npos == pos)
		? L""
		: fname.substr(0, pos);
}

wstring ExtractFileName(const wstring& fullPath)
{
	const size_t lastSlashIndex = fullPath.find_last_of(L"/\\");
	return fullPath.substr(lastSlashIndex + 1);
}