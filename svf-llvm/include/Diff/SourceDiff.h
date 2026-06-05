#ifndef SOURCE_DIFF_H
#define SOURCE_DIFF_H

#include "Util/Options.h"

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cstdlib>

using namespace std;

class sourceDiffInfo
{
public:
    sourceDiffInfo(int type, string file, int start, int end) : _type(type), _file(file), _start(start), _end(end)
    {
        // type: 1-add, 2-delete
        // file: fully qualified name
        // row: line num
    }

    inline int getType()
    {
        return _type;
    }
    inline string getFile()
    {
        return _file;
    }
    inline int getStart()
    {
        return _start;
    }
    inline int getEnd()
    {
        return _end;
    }

private:
    int _type;
    string _file;
    int _start;
    int _end;
};

class SourceDiffHandler
{
private:
    SourceDiffHandler(string sourceDiffFile, string _before, string _after) 
    {
        beforeDir = _before;
        afterDir = _after;
        sourceDiff = sourceDiffFile;

        auto getRootDir = [](string dir, string& root) -> void
        {
            /*if(dir[dir.length()-1] == '/')
            dir = dir.substr(0, dir.length()-1);
            int pos = dir.find_last_of("/");
            root = dir.substr(pos);*/
            root.clear();
            if (dir.empty())
                return;
            while (!dir.empty() && (dir.back() == '/' || dir.back() == '\\'))
                dir.pop_back();
            if (dir.empty())
                return;
            size_t pos = dir.find_last_of("/\\");
            if (pos == string::npos)
                root = dir;
            else
                root = dir.substr(pos);
        };

        getRootDir(beforeDir, beforeRootDir);
        getRootDir(afterDir, afterRootDir);

        vector<string> includedSuffix = {".h", ".hpp", ".c", ".cpp", ".cc", ".cbs", ".hbs"};
        string includedSuffixPattern = parseSuffix(includedSuffix);
        string cmd_find = "find " 
                        + beforeDir 
                        + " " 
                        + afterDir 
                        + R"( -type f| awk -F '/' '{print $NF}'|awk -F '.' '{print "*."$NF}'|sort|uniq|awk '{print $0}END{print ")" 
                        + includedSuffixPattern 
                        + R"("}'|sort|uniq -u > DiffPattern.txt)";
        string cmd_diff = "diff -N -r -X DiffPattern.txt " 
                        + beforeDir 
                        + " " 
                        + afterDir 
                        + " > " 
                        + sourceDiff;
        int ret_find = std::system(cmd_find.c_str());
        int ret_diff = std::system(cmd_diff.c_str());
    }

public:

    typedef map<string, vector<sourceDiffInfo>> FileToDiffMapTy;

    static SourceDiffHandler* getSourceDiffHandler()
    {
        if(sourceDiffHandler == nullptr)
        {
            sourceDiffHandler = new SourceDiffHandler(SVF::Options::sourcediff(), SVF::Options::beforecpp(), SVF::Options::aftercpp());
        }
        return sourceDiffHandler;
    }

    void handle();

    void parse(string, string);

    // void display();

    // void dump(string s);

    static inline string& getBeforeDir()
    {
        return beforeDir;
    }

    static inline string& getAfterDir()
    {
        return afterDir;
    }

    static inline string& getBeforeRootDir()
    {
        return beforeRootDir;
    }
    
    static inline string& getAfterRootDir()
    {
        return afterRootDir;
    }

    SourceDiffHandler::FileToDiffMapTy& getAddDiff()
    {
        return addDiff;
    }

    SourceDiffHandler::FileToDiffMapTy& getDeleteDiff()
    {
        return deleteDiff;
    }

private:
    string parseSuffix(vector<string> &includedSubfix) {
        string pattern = "";
        for(auto &E : includedSubfix)
        {
            string subfix = "";
            subfix += "*";
            subfix += E;
            subfix += R"(\n)";
            pattern += subfix;
            pattern += subfix;
        }
        return pattern;
    }

    bool hasSuffix(string& file, string& suffix)
    {
        int l1 = file.length();
        int l2 = suffix.length();
        if(l1 < l2) return false;

        return file.substr(l1 - l2) == suffix;
    }
    bool isSourceFile(string& file)
    {
        string pattern[] = {".h", ".hpp", ".c", ".cpp", ".cc", ".cbs", ".hbs"};
        for(string& suffix : pattern) {
            if(hasSuffix(file, suffix))
                return true;
        }
        return false;
    }

    static SourceDiffHandler* sourceDiffHandler;
    FileToDiffMapTy addDiff;
    FileToDiffMapTy deleteDiff;
    string sourceDiff;
    string before;
    string after;
    static string beforeDir;
    static string afterDir;
    static string beforeRootDir;
    static string afterRootDir;
};

#endif