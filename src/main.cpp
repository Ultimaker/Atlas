/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h>
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
#include <execinfo.h>
#include <sys/resource.h>
#endif
#include <stddef.h>
#include <vector>

#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "utils/string.h"

#include "modelFile/modelFile.h"
#include "settings.h"
#include "fffProcessor.h"



/*
#define ELPP_CUSTOM_COUT std::cerr
#include "easylogging++.h"
_INITIALIZE_EASYLOGGINGPP
#define _ELPP_STACKTRACE_ON_CRASH

void customCrashLogging(int sig) {
    LOG(ERROR) << "Application crashed!";
    el::Helpers::logCrashReason(sig, true);
    el::Loggers::flushAll();
    el::Helpers::crashAbort(sig); // FOLLOWING LINE IS ABSOLUTELY NEEDED AT THE END IN ORDER TO ABORT APPLICATION
    el::Loggers::flushAll();
}
*/

void print_usage()
{
    atlas::logError("usage: Atlas [-h] [-v] [-m 3x3matrix] [-c <config file>] [-s <settingkey>=<value>] -o <output.gcode> <model.stl>\n");
}

//Signal handler for a "floating point exception", which can also be integer division by zero errors.
void signal_FPE(int n)
{
    (void)n;
    atlas::logError("Arithmetic exception.\n");
    exit(1);
}

using namespace atlas;



#include "AABB_Tree.h"
void main_test(int argc, char **argv)
{
    std::cerr << " Program TEST begin... " << std::endl;

    AABB_Tree<int>::test();

}


int main(int argc, char **argv)
{
#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
    //Lower the process priority on linux and mac. On windows this is done on process creation from the GUI.
    setpriority(PRIO_PROCESS, 0, 10);
#endif

//    el::Helpers::setCrashHandler(customCrashLogging);
//    el::Configurations conf("LoggerConfig.conf");
//    el::Loggers::setDefaultConfigurations(conf,true); //Ensure all current (and future) loggers use the same settings
//    LOG(INFO) << "-----Application started-----" << std::endl;
//    el::Loggers::addFlag(el::LoggingFlag::AutoSpacing);


    //Register the exception handling for arithmic exceptions, this prevents the "something went wrong" dialog on windows to pop up on a division by zero.
    signal(SIGFPE, signal_FPE);

    fffProcessor processor;
    std::vector<std::string> files;

    logCopyright("Atlas version %s\n", VERSION);
    logCopyright("Copyright (C) 2014 Tim Kuipers\n");
    logCopyright("\n");
//    logCopyright("This program is free software: you can redistribute it and/or modify\n");
//    logCopyright("it under the terms of the GNU Affero General Public License as published by\n");
//    logCopyright("the Free Software Foundation, either version 3 of the License, or\n");
//    logCopyright("(at your option) any later version.\n");
//    logCopyright("\n");
//    logCopyright("This program is distributed in the hope that it will be useful,\n");
//    logCopyright("but WITHOUT ANY WARRANTY; without even the implied warranty of\n");
//    logCopyright("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n");
//    logCopyright("GNU Affero General Public License for more details.\n");
//    logCopyright("\n");
//    logCopyright("You should have received a copy of the GNU Affero General Public License\n");
//    logCopyright("along with this program.  If not, see <http://www.gnu.org/licenses/>.\n");

    processor.setSetting("supportType", "SUPPORT_TYPE_BLOCK");

    CommandSocket* commandSocket = NULL;

    for(int argn = 1; argn < argc; argn++)
    {
        char* str = argv[argn];
        if (str[0] == '-')
        {
            if (str[1] == '-')
            {
                //Long options
                if (stringcasecompare(str, "--socket") == 0)
                {
                    argn++;
                    commandSocket = new CommandSocket(atoi(argv[argn]));
                    processor.setCommandSocket(commandSocket);
                }else if (stringcasecompare(str, "--command-socket") == 0)
                {
                    commandSocket->handleIncommingData(&processor);
                }else if (stringcasecompare(str, "--") == 0)
                {
                    try {
                        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
                        if (files.size() > 0)
                            processor.processFiles(files);
                        files.clear();
                    }catch(...){
                        atlas::logError("Unknown exception\n");
                        exit(1);
                    }
                    break;
                }else{
                    atlas::logError("Unknown option: %s\n", str);
                }
            }else{
                for(str++; *str; str++)
                {
                    switch(*str)
                    {
                    case 'h':
                        print_usage();
                        exit(1);
                    case 'v':
                        atlas::increaseVerboseLevel();
                        break;
                    case 'p':
                        atlas::enableProgressLogging();
                        break;
                    case 's':
                        {
                            //Parse the given setting and store it.
                            argn++;
                            char* valuePtr = strchr(argv[argn], '=');
                            if (valuePtr)
                            {
                                *valuePtr++ = '\0';

                                processor.setSetting(argv[argn], valuePtr);
                            }
                        }
                        break;
                    default:
                        atlas::logError("Unknown option: %c\n", *str);
                        break;
                    }
                }
            }
        }else{
            files.push_back(argv[argn]);
        }
    }
    try {
        //Catch all exceptions, this prevents the "something went wrong" dialog on windows to pop up on a thrown exception.
        if (files.size() > 0)
            processor.processFiles(files);
    }catch(...){
        atlas::logError("Unknown exception\n");
        exit(1);
    }

//    el::Loggers::flushAll();
    //system("pause");
    std::cerr << " program finished! looping forever..." << std::endl;
    while(true) {} ;
    return 0;
}
