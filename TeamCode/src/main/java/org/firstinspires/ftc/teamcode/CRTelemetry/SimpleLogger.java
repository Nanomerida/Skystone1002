package org.firstinspires.ftc.teamcode.CRTelemetry;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class SimpleLogger {


    //The main project folder
    public static final String FTCLIB_FOLDER_NAME = "/FTCLib/";

    public static final File FTCLIB_FOLDER = new File(FTCLIB_FOLDER_NAME);

    //The folder for logging
    public static final String LOGGING_FOLDER_NAME = FTCLIB_FOLDER + "/Logs/";

    public static final File LOGGING_FOLDER = new File(LOGGING_FOLDER_NAME);

    //The maximum size of the logs
    private static final long LOG_DIR_MAX_SIZE = 25 * 1024 * 1024; // 25MB

    /**
     * Returns a file for use.
     *
     * <p>If the the file exists and overwrite is true, then the current file is deleted. If the
     * file exists and overwrite is false, then the a new file is created with ".1", ".2", etc appended
     * to the name. </p>
     *
     * @param dir The directory in which to look
     * @param fileName
     * @param overwrite
     * @return
     * @throws IOException
     */
    public static File createFileOnDevice(File dir, String fileName, boolean overwrite) throws IOException {

        if (!dir.exists()) {
            dir.mkdirs();
        }
        File file = new File(dir, fileName);
        // if file doesn't exists, then create it
        if (!file.exists() || overwrite) {
            if (file.exists() && overwrite)
                file.delete();
            file.createNewFile();
        } else {
            int i = 0;
            while (file.exists()) {
                file = new File(dir, fileName + "." + i);
                i++;
            }
            file.createNewFile();
        }
        return file;
    }


    private static void buildLogList(List<File> logFiles, File dir) {
        for (File file : dir.listFiles()) {
            if (file.isDirectory()) {
                buildLogList(logFiles, file);
            } else {
                logFiles.add(file);
            }
        }
    }

    private static void pruneLogsIfNecessary() {
        long totalSpace = LOGGING_FOLDER.getTotalSpace();
        List<File> logFiles = null;
        while (totalSpace > LOG_DIR_MAX_SIZE) {
            if (logFiles == null) {
                logFiles = new ArrayList<>();
                buildLogList(logFiles, LOGGING_FOLDER);
                Collections.sort(logFiles, (lhs, rhs) ->
                        Long.compare(lhs.lastModified(), rhs.lastModified()));
            }

            if (logFiles.size() == 0) break;
            File fileToRemove = logFiles.remove(0);
            totalSpace -= fileToRemove.getTotalSpace();
            //noinspection ResultOfMethodCallIgnored
            fileToRemove.delete();
        }
    }

    private static File getLogFile(String filename) throws  IOException {
        //noinspection ResultOfMethodCallIgnored
        LOGGING_FOLDER.mkdirs();

        pruneLogsIfNecessary();

        try {
            return createFileOnDevice(LOGGING_FOLDER, filename, false);
        } catch(IOException e){
            throw new IOException("An error occurred while retrieving log file!");
        }

    }



    //LOGGING STUFF BELOW

    /**
     * Name of the log file
     */
    private String logName;

    /**
     *
     */
    private ElapsedTime logTimer;

    /**
     * The accumulated log entries
     */
    private ArrayList<LogEntry> logEntries;


    /**Creates a logger with a given log name
     *
     * Time tracking is started on construction!
     *
     * @param logName The name to save log as.
     */
    public SimpleLogger(String logName){
        this.logName = logName;
        logEntries = new ArrayList<>();
        logTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * Creates  a logger with a given log name and time resolution
     * @param logName The name to save log as.
     * @param resolution The resolution to use for time
     */
    public SimpleLogger(String logName, ElapsedTime.Resolution resolution){
        this.logName = logName;
        logEntries = new ArrayList<>();
        logTimer = new ElapsedTime(resolution);
    }


    /**
     * Adds an entry to the log
     * @param title The entry title
     * @param data The text data
     */
    public void addEntry(String title, String data){
        logEntries.add(new LogEntry(logTimer.time(), title, data));
    }

    /**
     * Adds an entry to the log
     * @param logEntry Pre-made log entry
     */
    public void addEntry(LogEntry logEntry){
        logEntries.add(new LogEntry(logEntry));
    }

    /**
     * Returns a blank log entry object for the user
     * @return A new LogEntry
     */
    public LogEntry getBlankEntry(){
        return new LogEntry(0, "Uga Buga", "Uga Buga Buga!");
    }

    /**
     * Saves a log to fileName specified and with format specified
     * <p/>
     * This function will not overwrite an existing log file, but append ".1", ".2", etc. if it already exists
     *
     * @param fileType Format to write file in
     */
    public void saveAs(LogType fileType) {
        try {
            //Use correct filename for requested file type
            File f = getLogFile(logName);
            String out = "";
            switch (fileType) {
                case JSON:
                    Type logDataList = new TypeToken<List<LogEntry>>() {}.getType();
                    Gson g = SimpleGson.getInstance();
                    out = g.toJson(logEntries, logDataList);
                    break;
                case CSV:
                    out = "time,tag,data\n";
                    for (LogEntry l : logEntries) {
                        out += "\"" + l.time + "\"," + "\"" + l.title + "\"," + "\"" + l.data + "\"\n";
                    }
                    break;
                case TEXT:
                    for (LogEntry l : logEntries) {
                        out += l.time + ":[" + l.title + "]" + l.data + "\n";
                    }
                    break;
            }
            FileWriter fw = new FileWriter(f.getAbsoluteFile());
            BufferedWriter bw = new BufferedWriter(fw);
            bw.write(out);
            bw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Represents the file type of the log.
     *
     * Currently supports text files, json files, and csv files.
     */
    public enum LogType {

        JSON("json"),
        CSV("csv"),
        TEXT("txt"),
        XML("xml");

        private final String extension;

        LogType(String extension){
            this.extension = extension;
        }

        public String fullName(String fileName){
            return fileName + "." + extension;
        }

    }






    /**
     * Class that represents a log entry
     */
     class LogEntry {

        /**
        Time of the entry, measured as the elapsed time unless set
         */
        double time;

        /**
         * The title/tag of the entry
         */
        String title;

        /**
         * The text-based content of an entry
         */
        String data;

        public LogEntry(double time, String title, String data){
            this.time = time;
            this.title = title;
            this.data = data;
        }

        public LogEntry(LogEntry logEntry){
            time = logEntry.time;
            title = logEntry.title;
            data = logEntry.data;

        }
    }

}
