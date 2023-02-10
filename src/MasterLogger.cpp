#include "MasterLogger.h"

#include "FSCommon.h"

#include <cstdarg>
#include <cstring>
#include <cstdio>
#include <ctime>

void MasterLogger::writeString(const char* message, ...) {
    // 4K should be enough space for a log entry...
    const int messageMaxLength = 4000;
    char fmtMessage[messageMaxLength];
    memset(fmtMessage, 0, sizeof(fmtMessage));

    va_list arg_ptr;
    va_start(arg_ptr, message);
    int length = vsnprintf(fmtMessage, messageMaxLength - 1, message, arg_ptr);
    va_end(arg_ptr);
    fmtMessage[length] = '\n';

    // Open master file as write (defaults to appending)
    File masterFile = FSCom.open(MASTER_FILE_NAME, FILE_O_WRITE);
    masterFile.print(fmtMessage);
    masterFile.close();
}

void MasterLogger::writeData(LogData& data) {
    // 4K should be enough space for a log entry...
    const int messageMaxLength = 4000;
    char fmtMessage[messageMaxLength];
    memset(fmtMessage, 0, sizeof(fmtMessage));

    char timeString[100];
    struct tm ts = *localtime(&data.unixTimeStamp);
    strftime(timeString, sizeof(timeString), "%a %Y-%m-%d %H:%M:%S %Z", &ts);

    GeoCoord coord = data.gpsData;
    snprintf(
        fmtMessage, messageMaxLength,
        "[%s] %s activity detected at (%d° %d' %d\" %c, %d° %d' %d\" %c)\n",
        timeString, data.detectionType == LogData::DETECTION_TYPE_HUMAN ? "Human" : "Vehicular",
        coord.getDMSLatDeg(), coord.getDMSLatMin(), coord.getDMSLatSec(), coord.getDMSLatCP(),
        coord.getDMSLonDeg(), coord.getDMSLonMin(), coord.getDMSLonSec(), coord.getDMSLonCP()
    );

    // Open master file as write (defaults to appending)
    File masterFile = FSCom.open(MASTER_FILE_NAME, FILE_O_WRITE);
    masterFile.print(fmtMessage);
    masterFile.close();
}