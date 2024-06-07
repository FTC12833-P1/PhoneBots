package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class Renaldo_VuMarkIdentifier {

    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;
    private VuforiaTrackables relicTrackables;

    private LinearOpMode opMode;

    public Renaldo_VuMarkIdentifier(LinearOpMode opMode) {
        this.opMode = opMode;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters;
        if (opMode.getClass() == TeleOp_Renaldo.class) {
            parameters = new VuforiaLocalizer.Parameters();
        }
        else {
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        }
        parameters.vuforiaLicenseKey = "Acmakkv/////AAAAGXOA0cuKrEr7hpZ9JgE8SGBfqIBarylU/AFqu2jbyoBkqIk195TyB0KregRR/tMBd9C9366Y1UFqdmXSJr/zfPSONOtMd6Z+r/b6/MmRauAbn/Fjnu4ajUiEDDz3X36jg+mJb3ECZJNNhpMhsXMgADwOnYZvp+8RabsNtEsJubbIVZkjkmLvZ1cF+kyahJYwvYo4Tmf8InGkIfnPk1EikLJhWJmrwSePFOXVasDZw5bNCgIG7Yv+JfUXJ5opkgy0I3qHs9HW0oc4zOoa/z/PB1/Pf3HFDWRVBd7VPz9KUph6XJdkPjf11RWsOTvXQnm/4AH/6kOYqa7lhBHTAcLgLZEMfQZcxz77XV6SKr/2bjSd";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }

    public RelicRecoveryVuMark getVumark(){
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    public void activateTrackables(){
        relicTrackables.activate();
    }
}
