#include "log.h"
#include "stdio.h"
#include "stdarg.h"
#include "shell.h"

#if LOG_USING_COLOR == 1
#define memPrintHead \
    CSI(31)          \
    "    Offset: 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F" CSI(39)
#define memPrintAddr CSI(31) \
"0x%08x: " CSI(39)
#else
#define memPrintHead "    Offset: 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F"
#define memPrintAddr "0x%08x: "
#endif

Log *logList[LOG_MAX_NUMBER] = {0};
char logBuffer[LOG_BUFFER_SIZE];

#define LOG_LEVEL_TEXT(level) [level] = #level

char *logLevelText[] =
    {
        LOG_LEVEL_TEXT(LOG_NONE),
        LOG_LEVEL_TEXT(LOG_ERROR),
        LOG_LEVEL_TEXT(LOG_WRANING),
        LOG_LEVEL_TEXT(LOG_INFO),
        LOG_LEVEL_TEXT(LOG_DEBUG),
        LOG_LEVEL_TEXT(LOG_VERBOSE),
        LOG_LEVEL_TEXT(LOG_ALL)};

/**
 * @brief 注册log对象
 * 
 * @param log log对象
 */
void logRegister(Log *log)
{
    for (short i = 0; i < LOG_MAX_NUMBER; i++)
    {
        if (logList[i] == 0)
        {
            logList[i] = log;
            return;
        }
    }
}
SHELL_EXPORT_CMD(logRegister, logRegister, register log object);

/**
 * @brief 注销log对象
 * 
 * @param log log对象
 */
void logUnRegister(Log *log)
{
    for (short i = 0; i < LOG_MAX_NUMBER; i++)
    {
        if (logList[i] == log)
        {
            logList[i] = 0;
            return;
        }
    }
}
SHELL_EXPORT_CMD(logUnRegister, logUnRegister, unregister log object);

/**
 * @brief 设置log日志级别
 * 
 * @param log log对象
 * @param level 日志级别
 */
void logSetLevel(Log *log, LogLevel level)
{
    logAssert(log && log != LOG_ALL_OBJ, return );
    log->level = level;
}
SHELL_EXPORT_CMD(logSetLevel, logSetLevel, set log level\nlogSetLevel[log][level]);

/**
 * @brief 写数据到log对象
 * 
 * @param log Log对象
 * @param level log级别
 * @param buffer 数据
 * @param len 数据长度
 */
void logWriteObj(Log *log, LogLevel level, char *buffer, short len)
{
    if (log == LOG_ALL_OBJ)
    {
        for (short i = 0; i < LOG_MAX_NUMBER; i++)
        {
            if (logList[i] && logList[i]->active && logList[i]->level >= level)
            {
                logList[i]->write(buffer, len);
            }
        }
    }
    else if (log && log->active && log->level >= level)
    {
        log->write(buffer, len);
    }
}

/**
 * @brief log格式化写入数据
 * 
 * @param log log对象
 * @param level log级别
 * @param fmt 格式
 * @param ... 参数
 */
void logWrite(Log *log, LogLevel level, char *fmt, ...)
{
    va_list vargs;
    short len;
    static char lock = 0;

    while (lock)
    {
        // ut_delay(1);
    }
    lock = 1;
    va_start(vargs, fmt);
    len = vsnprintf(logBuffer, LOG_BUFFER_SIZE - 1, fmt, vargs);
    va_end(vargs);

    logWriteObj(log, level, logBuffer, len);
    lock = 0;
}

/**
 * @brief 16进制输出
 * 
 * @param log log对象
 * @param level log级别
 * @param base 内存基址
 * @param length 长度
 */
void logHexDump(Log *log, LogLevel level, void *base, unsigned int length)
{
    unsigned char *address;
    unsigned int len = length;

    if (length == 0)
    {
        return;
    }

    logWrite(log, level, "memory of 0x%08x, size: %d:\r\n", (unsigned int)base, length);

    address = (unsigned char *)((unsigned int)base & (~0x0000000F));
    length += (unsigned int)base - (unsigned int)address;
    length = (length + 15) & (~0x0000000F);

    logWrite(log, level, memPrintHead "\r\n");

    while (length)
    {
        logWrite(log, level, memPrintAddr, (unsigned int)address);
        for (int i = 0; i < 16; i++)
        {
            if ((unsigned int)(address + i) < (unsigned int)base || (unsigned int)(address + i) >= (unsigned int)base + len)
            {
                logWrite(log, level, "   ");
            }
            else
            {
                logWrite(log, level, "%02x ", *(address + i));
            }
        }
        logWrite(log, level, "| ");
        for (int i = 0; i < 16; i++)
        {
            if ((unsigned int)(address + i) < (unsigned int)base || (unsigned int)(address + i) >= (unsigned int)base + len)
            {
                logWrite(log, level, " ");
            }
            else
            {
                if (*(address + i) >= 32 && *(address + i) <= 126)
                {
                    logWrite(log, level, "%c", *(address + i));
                }
                else
                {
                    logWrite(log, level, ".");
                }
            }
        }
        logWrite(log, level, " |\r\n");
        address += 16;
        length -= 16;
    }
}
SHELL_EXPORT_CMD(hexdump, logHexDump, hex dump\nhexdump[log][lever][base][len]);

/**
 * @brief 格式化json写log
 * 
 * @param log log对象
 * @param level log级别
 * @param json json字符串
 */
void logWriteJson(Log *log, LogLevel level, char *json)
{
    short tab = 0;
    short quotation = 0;
    char *p = json;
    signed char newline = 0;
    while (*p)
    {
        newline = 0;
        if (*p == '{' || *p == '[')
        {
            tab++;
            newline = 1;
        }
        else if (*p == '}' || *p == ']')
        {
            tab--;
            newline = -1;
        }
        else if (*p == '\"')
        {
            quotation++;
        }
        else if (*p == ',')
        {
            if (!(quotation & 0x01))
            {
                newline = 1;
            }
        }

        if (newline == -1)
        {
            logWriteObj(log, level, "\r\n", 2);
            for (short i = 0; i < tab; i++)
            {
                logWriteObj(log, level, "    ", 4);
                // log->write("    ", 4);
            }
        }

        if (((quotation & 0x01) || *p != ' ') && *p != '\r' && *p != '\n')
        {
            logWriteObj(log, level, p, 1);
            if (*p == ':')
            {
                logWriteObj(log, level, " ", 1);
            }
        }

        if (newline == 1)
        {
            logWriteObj(log, level, "\r\n", 2);
            for (short i = 0; i < tab; i++)
            {
                logWriteObj(log, level, "    ", 4);
            }
        }

        p++;
    }
    logWriteObj(log, level, "\r\n", 2);
}
