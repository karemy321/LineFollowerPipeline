/**
 * ============================================================================
 *  Blockly block — "line follower [Steer / Power]"   (SENSOR / REPORTER)
 *  Backed by: LineFollowerPipeline.java
 * ============================================================================
 *
 *  This is a SENSOR block, not an action block.
 *    - NO inputs.
 *    - NO statement connections (it does not sit in the script stack).
 *    - It has an OUTPUT plug, so it drops into any numeric socket —
 *      e.g. the steering / power sockets of "move steering".
 *
 *  It behaves exactly like the built-in "reflected Light" sensor block:
 *  a dropdown selects WHICH value is reported.
 *
 *  WHY A DROPDOWN AND NOT TWO OUTPUT PLUGS:
 *    Blockly blocks have exactly ONE output connector. That is a hard limit
 *    of the library. To report two values you either use two blocks, or one
 *    block with a dropdown. The dropdown matches the existing sensor blocks,
 *    so it is the option used here.
 *
 *  THE STUDENT DOES THE MATH:
 *    The block just reports the raw pipeline value. The student plugs it into
 *    "move steering" and can scale/clamp it however they like:
 *
 *        move steering  on  steering ( line follower[Steer] × Ks )
 *                           power    ( line follower[Power] × Kp )
 * ============================================================================
 */

/* ---------------------------------------------------------------------------
 * 1. BLOCK DEFINITION
 * ------------------------------------------------------------------------ */
Blockly.defineBlocksWithJsonArray([
  {
    "type": "line_follower_read",
    "message0": "line follower %1",
    "args0": [
      {
        "type": "field_dropdown",
        "name": "VALUE",
        "options": [
          ["Steer", "STEER"],
          ["Power", "POWER"]
        ]
      }
    ],
    "inputsInline": true,

    // OUTPUT — no previousStatement / nextStatement.
    // This is what makes it a reporter that plugs into a socket.
    "output": "Number",

    "colour": "#CA8A4D",   // tan — matches the built-in sensor blocks
    "tooltip": "Reads one camera frame and reports a line-following value. Steer: -100 to +100 (0 = centred). Power: 0 to 100 (100 = straight ahead, drops before turns, 0 = line lost).",
    "helpUrl": "https://github.com/YOUR_USERNAME/YOUR_REPO"
  }
]);


/* ---------------------------------------------------------------------------
 * 2. CODE GENERATORS
 *
 *    A reporter returns [code, precedence] — NOT a string.
 *    ORDER_FUNCTION_CALL because the emitted code is a method call.
 * ------------------------------------------------------------------------ */

// --- Java ---
Blockly.Java = Blockly.Java || new Blockly.Generator('Java');

Blockly.Java.forBlock['line_follower_read'] = function (block, generator) {
  const which = block.getFieldValue('VALUE');   // "STEER" or "POWER"
  const code  = (which === 'STEER')
    ? 'LineFollowerHelper.getSteer()'
    : 'LineFollowerHelper.getPower()';
  return [code, generator.ORDER_FUNCTION_CALL];
};

// --- JavaScript ---
Blockly.JavaScript.forBlock['line_follower_read'] = function (block, generator) {
  const which = block.getFieldValue('VALUE');
  const code  = (which === 'STEER')
    ? 'lineFollower.getSteer()'
    : 'lineFollower.getPower()';
  return [code, generator.ORDER_FUNCTION_CALL];
};


/* ---------------------------------------------------------------------------
 * 3. TOOLBOX ENTRY
 * ------------------------------------------------------------------------ */
/*
  <category name="Line Following" colour="#CA8A4D">
    <block type="line_follower_read">
      <field name="VALUE">STEER</field>
    </block>
    <block type="line_follower_read">
      <field name="VALUE">POWER</field>
    </block>
  </category>

  Listing it twice puts a ready-made Steer block AND a ready-made Power block
  in the flyout, so the student can grab either without touching the dropdown.
*/


/* ---------------------------------------------------------------------------
 * 4. RUNTIME HELPER  (Java side)
 *
 *    IMPORTANT — ONE FRAME, ONE PROCESS CALL:
 *    getSteer() and getPower() are usually called twice in the SAME statement
 *    (once for the steering socket, once for the power socket). The pipeline
 *    must NOT run twice on the same frame — that would double the cost and
 *    could return values taken from two different frames.
 *
 *    So the helper caches: the first call in a frame processes, the second
 *    call reads the cached result. Call newFrame() once per camera frame.
 * ------------------------------------------------------------------------ */
/*
  ------------------------------------------------------------------------
  public final class LineFollowerHelper {

      private static LineFollowerPipeline pipeline;
      private static Camera camera;

      private static byte[]  frame;
      private static int     steer;
      private static int     power;
      private static boolean processedThisFrame = false;

      // --- Called once at startup ---
      public static void init(Camera camera) {
          pipeline = new LineFollowerPipeline();

          LineFollowerPipeline.Config cfg = new LineFollowerPipeline.Config();
          cfg.trackIsBright    = true;    // white line on dark floor
          cfg.cameraTiltDeg    = 45f;     // phone mount angle
          cfg.bottomIgnoreFrac = 0.20f;   // hide robot body from scan
          cfg.trackMemory      = true;    // ignore reflections
          pipeline.configure(cfg);

          LineFollowerHelper.camera = camera;
      }

      // --- Calibration: feed frames until this returns true ---
      public static boolean calibrate() {
          byte[] f = camera.getGrayscaleFrame();
          return pipeline.calibrate(f, camera.getWidth(), camera.getHeight());
      }

      // --- Call ONCE at the top of every loop iteration ---
      public static void newFrame() {
          frame = camera.getGrayscaleFrame();
          processedThisFrame = false;
      }

      private static void ensureProcessed() {
          if (processedThisFrame) return;
          int[] r = pipeline.processFrame(frame, camera.getWidth(), camera.getHeight());
          steer = r[0];   // -100 ... +100
          power = r[1];   //    0 ... 100
          processedThisFrame = true;
      }

      // --- What the Blockly block compiles down to ---
      public static int getSteer() { ensureProcessed(); return steer; }
      public static int getPower() { ensureProcessed(); return power; }

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
  Blocks:

    repeat forever
      move steering  on  steering ( line follower[Steer] )
                         power    ( line follower[Power] )

  Generated Java:

    while (true) {
        LineFollowerHelper.newFrame();
        robot.moveSteering(LineFollowerHelper.getSteer(),
                           LineFollowerHelper.getPower());
    }

  And because it is just a number, the student can do their own math with it,
  exactly like the hand-built version:

      steering ( line follower[Steer] × Ks )
      power    ( line follower[Power] × Kp )
*/
