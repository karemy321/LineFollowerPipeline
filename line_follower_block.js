/**
 * ============================================================================
 *  Blockly block — "follow the line at speed [N] %"
 *  Backed by: LineFollowerPipeline.java
 * ============================================================================
 *
 *  ONE input: speed (0-100).
 *
 *  Why only speed?
 *    - Steering is NOT a choice. There is exactly one correct steering value
 *      at any moment, and the pipeline already computes it. Exposing it would
 *      only give students a way to break the robot.
 *    - Speed IS a choice. It's the strategy knob: go fast and risk flying off
 *      a corner, or go slow and finish clean. That's the thing worth teaching.
 *
 *  IMPORTANT — how speed is applied:
 *    result[1] (Tracking) is NOT a speed. It's a 0-100 straightness signal
 *    that DROPS toward 0 as a curve approaches. So the student's speed does
 *    not REPLACE it — it SCALES it:
 *
 *        actualSpeed = tracking * studentSpeed / 100
 *
 *    Result: at "speed 80" the robot runs at 80 on straights but STILL
 *    automatically brakes into turns. The student gets one simple dial and
 *    the smart cornering comes for free.
 * ============================================================================
 */

/* ---------------------------------------------------------------------------
 * 1. BLOCK DEFINITION
 * ------------------------------------------------------------------------ */
Blockly.defineBlocksWithJsonArray([
  {
    "type": "line_follower_follow",
    "message0": "follow the line on power %1",
    "args0": [
      {
        "type": "input_value",
        "name": "POWER",
        "check": "Number"
      }
    ],
    "inputsInline": true,
    "previousStatement": null,
    "nextStatement": null,
    "colour": 30,               // orange — motor / movement family
    "tooltip": "Drives along the line. Power 0-100 sets how fast to go on straights. Steering is automatic, and the robot still slows down by itself for curves.",
    "helpUrl": "https://github.com/YOUR_USERNAME/YOUR_REPO"
  }
]);


/* ---------------------------------------------------------------------------
 * 2. CODE GENERATORS
 * ------------------------------------------------------------------------ */

// --- Java ---
Blockly.Java = Blockly.Java || new Blockly.Generator('Java');

Blockly.Java.forBlock['line_follower_follow'] = function (block, generator) {
  const power = generator.valueToCode(block, 'POWER', 0) || '50';
  return 'LineFollowerHelper.followLine(' + power + ');\n';
};

// --- JavaScript ---
Blockly.JavaScript.forBlock['line_follower_follow'] = function (block, generator) {
  const power = generator.valueToCode(block, 'POWER', 0) || '50';
  return 'lineFollower.followLine(' + power + ');\n';
};


/* ---------------------------------------------------------------------------
 * 3. TOOLBOX ENTRY
 * ------------------------------------------------------------------------ */
/*
  Note the <shadow> — it puts a default "50" in the socket, so the block
  works the moment a student drags it out, with nothing plugged in.

  <category name="Line Following" colour="30">
    <block type="line_follower_follow">
      <value name="POWER">
        <shadow type="math_number">
          <field name="NUM">50</field>
        </shadow>
      </value>
    </block>
  </category>
*/


/* ---------------------------------------------------------------------------
 * 4. RUNTIME HELPER  (Java side)
 * ------------------------------------------------------------------------ */
/*
  ------------------------------------------------------------------------
  public final class LineFollowerHelper {

      private static LineFollowerPipeline pipeline;
      private static Camera camera;
      private static Robot  robot;

      // Called once when the program starts
      public static void init(Camera camera, Robot robot) {
          pipeline = new LineFollowerPipeline();

          LineFollowerPipeline.Config cfg = new LineFollowerPipeline.Config();
          cfg.trackIsBright    = true;    // white line on dark floor
          cfg.cameraTiltDeg    = 45f;     // phone mount angle
          cfg.bottomIgnoreFrac = 0.20f;   // hide robot body from scan
          cfg.trackMemory      = true;    // ignore reflections
          pipeline.configure(cfg);

          LineFollowerHelper.camera = camera;
          LineFollowerHelper.robot  = robot;
      }

      // This is what the Blockly block compiles down to.
      // maxPower: 0-100, chosen by the student.
      public static void followLine(int maxPower) {
          if (maxPower < 0)   maxPower = 0;
          if (maxPower > 100) maxPower = 100;

          byte[] frame  = camera.getGrayscaleFrame();
          int[]  result = pipeline.processFrame(frame, camera.getWidth(), camera.getHeight());

          int steering = result[0];   // -100 ... +100  (auto)
          int tracking = result[1];   //    0 ... 100   (auto, drops before turns)

          // Scale, don't replace: keeps the automatic braking on curves.
          int power = tracking * maxPower / 100;

          robot.setSteeringAngle(steering);
          robot.setSpeed(power);
      }

      public static void release() {
          if (pipeline != null) pipeline.release();
      }
  }
  ------------------------------------------------------------------------
*/


/* ---------------------------------------------------------------------------
 * 5. WHAT THE STUDENT BUILDS  ->  WHAT IT COMPILES TO
 * ------------------------------------------------------------------------ */
/*
  Blocks:                                 Generated Java:

  ┌────────────────────────────────┐
  │ repeat for t = 60 sec          │      long end = now() + 60_000;
  │  ┌──────────────────────────┐  │      while (now() < end) {
  │  │ follow the line on power  │  │          LineFollowerHelper.followLine(80);
  │  │            (80) %        │  │      }
  │  └──────────────────────────┘  │
  └────────────────────────────────┘

  At power 80:
    straight  -> tracking 100 -> drives at 80
    curve     -> tracking  40 -> drives at 32   (slows down by itself)
    line lost -> tracking   0 -> stops
*/
