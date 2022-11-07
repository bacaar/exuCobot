/*
 * @author:	Aaron Bacher
 * @date:	2022-10-25
 *
 * @brief:	Logger module for text and csv logging
 *          provides possibility for multithreading
 *
 * @note:	-
 *
 */

#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <memory>
#include <ctime>

#include <iostream>

#include <unistd.h>
#include <boost/filesystem.hpp>

#define ENABLE_MULTITHREADING 1

#if ENABLE_MULTITHREADING
#include <mutex>
#include <thread>
#include <chrono>
#endif

class LogEntry{
public:
    virtual void constructEntry() = 0;
    std::string getEntry() { return m_entry; }
protected:
    std::string m_entry;
};

// logger class
class Logger{
public:
    Logger(std::string logFileName, bool enableConsolePrinting=false)
            :m_enableConsolePrinting(enableConsolePrinting), m_isHandledByThreader(false)
    {}

    ~Logger(){
        m_logFile.close();

        // remove logfile from s_openLogFiles
        //s_openLogFiles.erase(std::find(s_openLogFiles.begin(), s_openLogFiles.end(), m_logFilePath));
    }

    bool isHandledByThreader() { return m_isHandledByThreader; }

protected:

    bool m_enableConsolePrinting;

    std::string m_logFilePath;
    std::ofstream m_logFile;

    // method to intialize file
    void setup(std::string logFileName){

        // check if log directory exists and create it if not
        char cwd[256];
        getcwd(cwd, 256);

#ifdef __linux__
        std::string logFileDir = std::string(cwd) + "/log";
        m_logFilePath = logFileDir + std::string("/") + logFileName;
#elif _WIN32
        std::string logFileDir = std::string(cwd) + "\\log";
            m_logFilePath = logFileDir + std::string("\\") + logFileName;
#endif

        /*
        // check that no other logger is already writing to that file
        if (std::find(s_openLogFiles.begin(), s_openLogFiles.end(), m_logFilePath) != s_openLogFiles.end()){
            std::string errMsg = "-----------\nERROR: log-file already in use by other Logger!\n-----------";
            printToConsole(errMsg);
        }
        else{
            s_openLogFiles.push_back(m_logFilePath);
        }
         */

        if (!boost::filesystem::exists(logFileDir)) {
            boost::filesystem::create_directory(logFileDir);
        }

        if(m_logFilePath.substr(m_logFilePath.size()-4, 4) == ".log"){          // append mode
            m_logFile.open(m_logFilePath, std::ios::out | std::ios::app);
        }
        else if(m_logFilePath.substr(m_logFilePath.size()-4, 4) == ".csv"){     // delete file content and write afterwards
            m_logFile.open(m_logFilePath, std::ios::out);
        }
        else{
            std::string errMsg = "-----------\nERROR: Unknown log-file type!\n-----------";
            printToConsole(errMsg);
        }

        // check if logfile creation and opening as been successful
        if(!m_logFile.is_open()){
            std::string errMsg = "-----------\nERROR: Could not open log-file!\n-----------";
            printToConsole(errMsg);
        }
        else{
            std::string infoMsg = "opened log file " + m_logFilePath;
            printToConsole(infoMsg);
        }
    }

    // handle printing to console with time stamps etc.
    void printToConsole(const std::string& msg){

        // note: I used to differentiate between error messages (then printed with cerr <<) and non-error messages (printed with cout <<)
        // but as this might affect printing order (which was initially the objective of differentiating), I removed it
        std::cout << msg << std::endl;
    }

    // prints entry to logfile
    void printToFile(const std::string& msg){

        // print to file
        m_logFile << msg << std::endl;
    }

private:

    bool m_isHandledByThreader;       // defines if logger should work on its own (false) or if logging is done by LogThreader in separate thread (true)

    //static std::vector<std::string> s_openLogFiles; // holds paths to all currently open log files in order to make sure that not two loggers are writing to same one

    // construct entry, give command to write to console and/or file
    virtual void print(std::unique_ptr<LogEntry> entry, bool enforceConsoleWriting=false) = 0;


#if ENABLE_MULTITHREADING
public:
    // removes first item in queue and passes it to caller (e.g. LogThreader) (returns nullptr if queue empty)
    std::unique_ptr<LogEntry> getQueueItem(){

        std::unique_ptr<LogEntry> entry;

        m_queueMutex.lock();
        if(m_logEntries.size() == 0){
            return std::unique_ptr<LogEntry>(nullptr);
        }
        else{
            entry = move(m_logEntries.front());
            m_logEntries.pop();
        }
        m_queueMutex.unlock();
        return move(entry);
    }

    int getQueueSize(){
        int size;

        // needed for stability, else it often pauses on exception during runtime (and for a try-catch block I would need a exception transport system between the threads
        // with which I don't want to be bothered right now)
        std::this_thread::sleep_for(std::chrono::nanoseconds(1));

        m_queueMutex.lock();
        size =  m_logEntries.size();
        m_queueMutex.unlock();

        return size;
    }

protected:
    std::mutex m_queueMutex;        // controls queue access

    std::queue<std::unique_ptr<LogEntry>> m_logEntries;      // if handled by LogThreader, log-method writes into this buffer instead of writing directly to file and/or console

private:
    // LogThreader needs to access print function
    friend class LogThreader;

#endif

};

// selection from https://www.ibm.com/docs/en/cognos-analytics/10.2.2?topic=SSEP7J_10.2.2/com.ibm.swg.ba.cognos.ug_rtm_wb.10.2.2.doc/c_n30e74.html
enum LogLevel {
    Error,
    Warning,
    Info,
    Debug
};

inline bool logLevelToStr(std::string& str, LogLevel logLevel){
    switch (logLevel)
    {
        case LogLevel::Error:
            str = "ERROR";
            return true;
        case LogLevel::Warning:
            str = "WARNING";
            return true;
        case LogLevel::Info:
            str = "INFO";
            return true;
        case LogLevel::Debug:
            str = "DEBUG";
            return true;
    }

    str = "UNDEFINED";
    return false;
}

/* Derived Logger class to represend log-entries in normal text log */
class LogEntryText : public LogEntry{
public:
    LogEntryText(LogLevel logLevel,
                 std::string msg,
                 std::string customTimeStr = "",
                 time_t rawTime = 0)
            :m_logLevel(logLevel), m_msg(msg), m_customTimeStr(customTimeStr), m_rawTime(rawTime)
    {}

    void constructEntry() override{

        m_entry = m_msg;

        // add logLevel as string at front
        addLogLevel(m_entry);

        // add time as string at front
        addTime(m_entry);
    }

private:
    // adds log Level as string in front of message
    void addLogLevel(std::string& msg){

        // create empty string and write LogLevel into it
        std::string levelStr = "";
        logLevelToStr(levelStr, m_logLevel);

        // add colon
        levelStr += ":";

        // all log entries should have same length, so fill up space
        // TODO does not work properly for Warnings
        for(int i = 0; i < 12-levelStr.length(); ++i){
            levelStr += " ";
        }

        // combine everything
        msg = levelStr + msg;
    }

    // adds time as string in front of message
    void addTime(std::string& msg){

        // custom time
        std::string customTimeStr = "";
        if(m_customTimeStr != ""){
            // TODO format
            if(m_customTimeStr != " "){
                msg = m_customTimeStr + " - " + msg;
            }
        }
        else{
            // real time
            std::string rawTimeStr = "";
            if(m_rawTime == 0){
                time(&m_rawTime);
            }
            struct tm * timeinfo;
            char buffer[80];
            timeinfo = localtime(&m_rawTime);

            strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
            rawTimeStr = std::string(buffer);
            // TODO format
            msg = rawTimeStr + " - " + msg;
        }
    }

    LogLevel m_logLevel;
    std::string m_msg;
    std::string m_customTimeStr = "";
    time_t m_rawTime = 0;
};

/* Derived Logger class to handle text logging (normal .log-files) */
class TextLogger : public Logger{
public:
    TextLogger(std::string logFileName, LogLevel newLogLevel, bool enableConsolePrinting=false, bool useCustomTime=false)      // for normal text logs
            :Logger(logFileName, enableConsolePrinting), m_logLevel(newLogLevel), m_useCustomTime(useCustomTime)
    {
        // if no specific logFileName provided, use default
        if(logFileName == ""){
            logFileName = "log0.log";
        }

        // if logfile has no file extension, add it
        if(logFileName.substr(logFileName.size()-4, 4) != ".log"){
            logFileName += ".log";
        }

        setup(logFileName);

        // check if passed log-level is valid (and get loglevel as string), else return
        std::string levelStr = "";
        if(!logLevelToStr(levelStr, m_logLevel)){
            std::string errMsg = "Undefined LogLevel. Logger is terminating.";
            std::unique_ptr<LogEntryText> entry = std::make_unique<LogEntryText>(LogLevel::Error, errMsg, "", 0);
            print(move(entry));
            return;
        }

        std::string initMsg = "Starting logger with log level " + levelStr;
        std::unique_ptr<LogEntryText> entry = std::make_unique<LogEntryText>(LogLevel::Info, initMsg, "", 0);
        print(move(entry));

    }

    ~TextLogger(){
        std::string infoMsg = "TextLogger has been shut down";

        std::unique_ptr<LogEntryText> entry = std::make_unique<LogEntryText>(LogLevel::Info, infoMsg, "", 0);
        print(move(entry), true);
        m_logFile << "------------------------------------------\n\n";
    }

    // create log entries
    // write entries to m_logEntries
    void log(const std::string &logEntry, LogLevel logLevel, std::string timeStr = ""){

        if(logLevel <= m_logLevel) {

            time_t rawTime = 0;
            if(!m_useCustomTime){
                time (&rawTime);
            }

            std::unique_ptr<LogEntryText> entry = std::make_unique<LogEntryText>(logLevel, logEntry, timeStr, rawTime);

            if(isHandledByThreader()){
#if ENABLE_MULTITHREADING
                // write to queue
                m_queueMutex.lock();
                m_logEntries.push(move(entry));
                m_queueMutex.unlock();
#endif
            } else {
                // write to log
                print(move(entry));
            }
        }
    }

    // wrapper for above method
    void log(const char* logEntry, LogLevel logLevel, std::string timeStr = ""){
        std::string msg(logEntry);  // convert char* to std::string
        log(msg, logLevel, timeStr);
    }

    // allows to change loglevel after logger-construction.
    // e.g. to increase loglevel temporarely
    void setLogLevel(LogLevel newLogLevel){
        m_logLevel = newLogLevel;
    }

private:

    LogLevel m_logLevel;

    bool m_useCustomTime;

    // construct entry, give command to write to console and/or file
    void print(std::unique_ptr<LogEntry> entry, bool enforceConsoleWriting=false) override{

        entry->constructEntry();
        std::string msg = entry->getEntry();

        // make entries at different locations
        if(m_enableConsolePrinting || enforceConsoleWriting){
            printToConsole(msg);
        }

        printToFile(msg);
    }
};


/* Derived Logger class to represend log-entries in normal text log */
class LogEntryCSV : public LogEntry{
public:
    LogEntryCSV(const std::string& content)
            :m_content(content)
    {}

    void constructEntry() override{
        m_entry = m_content;
    }

private:
    std::string m_content;
};

/* Derived Logger class to handle text logging (normal .log-files) */
class CsvLogger : public Logger{
public:
    CsvLogger(std::string logFileName):Logger(logFileName, false){
        // if no specific logFileName provided, use default
        if(logFileName == ""){
            logFileName = "csv0.csv";
        }

        // if logfile has no file extension, add it
        if(logFileName.substr(logFileName.size()-4, 4) != ".csv"){
            logFileName += ".csv";
        }

        setup(logFileName);
    }

    ~CsvLogger(){

        std::string infoMsg = "CsvLogger has been shut down";
        printToConsole(infoMsg);

    }

    // create log entries
    // write entries to m_logEntries
    void log(const std::string &logEntry){

        std::unique_ptr<LogEntryCSV> entry = std::make_unique<LogEntryCSV>(logEntry);
        if(isHandledByThreader()){
#if ENABLE_MULTITHREADING
            // write to queue
            m_queueMutex.lock();
            m_logEntries.push(move(entry));
            m_queueMutex.unlock();
#endif
        } else {
            // write to log
            print(move(entry));
        }
    }

    void log(const char* logEntry){
        std::string msg(logEntry);      // convert char* to std::string
        log(msg);
    }

private:

    // construct entry, give command to write to console and/or file
    void print(std::unique_ptr<LogEntry> entry, bool enforceConsoleWriting=false) override{

        entry->constructEntry();
        std::string msg = entry->getEntry();

        // check amount of columns
        if(m_nColumns == 0){    // first time
            for (int i = 0; i < msg.size(); ++i){
                if(msg[i] == ',') ++m_nColumns;
            }
            ++m_nColumns;   // there is always one more column than semicolons

        }
        else {  // every other time
            int count = 1;  // there is always one more column than semicolons
            for (int i = 0; i < msg.size(); ++i){
                if(msg[i] == ',') ++count;
            }
            if(count != m_nColumns){
                std::string warningMsg = "WARNING! Amount of columns (" + std::to_string(count) + ") does not correspond to columns in first row (" + std::to_string(m_nColumns) + ")";
                printToConsole(warningMsg);
            }
        }

        if(enforceConsoleWriting) printToConsole(msg);

        printToFile(msg);
    }

    // amount of columns; gets set on first entry according to amount of semicolons
    int m_nColumns = 0;
};

#if ENABLE_MULTITHREADING
/* Class to handle multiple log-files in single thread */
class LogThreader {
public:
    LogThreader(){
        m_loggerRunning = true;
        m_thread = std::thread(&LogThreader::logging, this);
    }

    ~LogThreader(){
        m_loggerRunning = false;
        m_thread.join();   // wait for logging method to finish
        std::cout << "LogThreader has been shut down" << std::endl;
    }

    // add new logger to be handled
    void addLogger(std::shared_ptr<Logger> logger){
        m_handledLoggers.push_back(logger);
        logger->m_isHandledByThreader = true;
    }

private:
    // runs in separate thread; continuously writes entries of m_logEntries to file and/or console
    void logging(){

        while(true){
            bool canExit = !m_loggerRunning;    // if logger is still running, no chance for exit

            for(int i = 0; i < m_handledLoggers.size(); ++i){

                // do one item per queue per iteration, so that every logger gets processed equally
                int size = m_handledLoggers[i]->getQueueSize();
                if(size > 0){
                    std::unique_ptr<LogEntry> entry = m_handledLoggers[i]->getQueueItem();

                    // wirte to console and/or file
                    m_handledLoggers[i]->print(move(entry));
                    size--;
                }
                if(size > 0) canExit = false;   // if m_loggerRunning is false, then even if only one logger has some items in it's queue, a new iteration is needed
            }

            if(canExit){
                break;
            }
        }
    }

    bool m_loggerRunning;                   // is set to true by constructor and to false by destructor; keeps logging() function running

    std::thread m_thread;                   // logging thread: started in constructor, joined in destructor

    std::vector<std::shared_ptr<Logger>> m_handledLoggers;
};
#endif
