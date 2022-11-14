package org.team498.lib.math;

import java.io.IOException;
import org.team498.C2022.Constants;
import org.team498.lib.logging.CSVWriter;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TrajectoryGenerator extends InstantCommand {
    private final Curve curve;// = new Curve(1, 1, 1, 0, 0, 0);
    private CSVWriter writer;
    public TrajectoryGenerator(String fileName, Curve curve) {
        this.curve = curve;
        writer = new CSVWriter(Constants.kRoborioTrajectoryFilepath, fileName);
    }
    @Override
    public void execute() {
        try {
            generate();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void generate() throws IOException {
        writer.open();
        for (int i = 0; i < curve.length; i++) {
            printInfo(
                curve.positions.get(i)[0], 
                curve.positions.get(i)[1], 
                curve.positions.get(i)[2], 
                curve.positions.get(i)[3]
            );
        }
        writer.close();
    }
    private void printInfo(double timeStamp, double x, double y, double r) {
		writer.write(String.valueOf(timeStamp), String.valueOf(x), String.valueOf(y), String.valueOf(r));
	}

}
