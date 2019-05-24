package util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;


/**
 * Logger
 */
public class Logger {

     public static void write(String name, String data) {
         try (PrintWriter writer = new PrintWriter(new FileWriter(name + ".csv", true))) {
//             writer.print(new Date().toString());

             if (data != null) {
                 writer.print(data);
                 writer.print(", ");
             }

             writer.println();
         } catch (IOException e) {
             e.printStackTrace();
         }
     }
    
}