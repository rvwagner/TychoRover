/*
Parts manifest:
- Breadboard jumper cables (square ends).  You will need both male->male and
  male->female (and ideally a couple female->female ones to cut up to make splitters)
- DROK four-channel speed controller - https://www.amazon.com/gp/product/B017FZF42G
- Arduino Uno
- 4x TowerPro SG90 micro servos (or equivalent cheap knock-offs) - https://www.amazon.com/gp/product/B015H5AVZG
- 4x extra screws of the same size as the mounting screws that come with the servos
- 4x 12V micro gearmotors (I used 175rpm ones) - https://www.sparkfun.com/products/12205
- 8-12V battery (I used a 3S LiFePO4 9.9V battery rated at 5C draw and 1800mAh capacity) - https://www.amazon.com/gp/product/B00URCLJ6U
- Suitable connector for your battery (I converted mine to use Anderson PowerPoles)
- Light-duty wire for powering motors (I used 22AWG Sparkfun hookup wire) - https://www.sparkfun.com/products/8023
- Mini zip-ties (DIMENSIONS)
- 3D printer- 


Assembly instructions:
1) Print out all parts (see end of file).  You may wish to start by printing one of
   each part (epecially the Motor Grippers) to check the fit for your printer.
1x Frame
2x Front Beams (or 1x Front Beam and 1x Front Beam With Pi Mount)
4x Motor Grippers (check the printing path preview to make sure that there is only a single layer between the servo horn cavity on the bottom and the screw-hole above it)
4x Wheels
4x Wheel Grippers
Assorted mounting hardware and wire management clips as desired

2) Slot Front Beams onto Frame (may need either mallet or glue, depending on fit).
   Wire management loops should be on on the bottom, servo mounts on the top (the
   side of the main frame with the large flat plate is the bottom).

3) Screw micro servos onto Front Beams.  Run the servo wires through the opening
   where the Frame meets the Front Bar from bottom to top.

4) Carefully(!) slide motors into the Motor Grippers.  If a side clip snaps off,
   you may want to re-print the gripper (they are a bit fragile).  Whether you
   reprint after failures or not, anchor the motor in place using a small zip-tie
   wrapped around the clips in the slot provided.

5) Insert a small single-armed servo horn into the circular opening on the
   bottom of each Motor Gripper.  It should slide all the way in and snap into
   a position entirely recessed into the part.

6) Push a horn+Gripper onto each servo, and make sure that when the Gripper is
   in-line with the Front Beam, the servo is near the center of its range.  The
   Gripper should be able to turn 90 degrees in each direction.  Screw the
   Gripper to the servo using a longer screw than the small bolts provided- the
   mounting screws worked nicely on my servos, although I then needed to get
   extra screws to mount the servos to the frame.

7) For each motor, strip one end of a pair of ~16" wires, and run the stripped
   end through the wire management loop on the Front Bar closest to the motor,
   and then through the loop on the gripper.  Solder the wires to the motor.
   You may want to use a dab of hot glue to anchor the wires in place at the
   Gripper wire management loop to eliminate all strain on the motor conections.
   Run the other end of the wires up between the two bars that run the length
   of the Frame.

8) Push the mounting brackets into the mounting holes in the corners of the
   speed controller board, and clip it onto one side of the frame, with the
   motor outputs pointing towards the center fo the frame.  Do the same with
   the Arduino, with the digital outputs towards the center of the frame.

9) Trim motor power wires to length, strip the ends, and insert them into
   appropriate channels on the motor controller.  I used the following assignments:
Out1/2: Back Left
Out3/4: Back Right (reverse the + and - wires relative to the left motors)
Out5/6: Front Left
Out7/8: Front Right (reverse the + and - wires relative to the left motors)

10: Solder up four splitter jumper wires, with one male end and two female
    ends.  These will control the forward/backward pins on the speed
    controller (using two arduino pins for the left motors and two for the
    right).  I used the following pin connections:
Arduino    DROK
   2   -> IN1/IN5
   4   -> IN2/IN6
   7   -> IN3/IN7
   8   -> IN4/IN8

11: Using normal male->female jumper cables, connect Arduino PWM pins to
    the DROK enable pins, and power the Arduino from the DROK's 5V regulator:
Arduino    DROK
   3   -> ENA (channels 1-2)
   5   -> ENB (channels 3-4)
   6   -> ENA (channels 5-6)
  11   -> ENB (channels 7-8) (note that the Servo library disables PWM on pins 9 and 10)
 VIN   -> +5 (I used the one next to IN1)
 GND   -> GND

12) Connect a connector for your battery to the VCC and GND screw terminals
    on the inboard side of the DROK board.  At this point you can plug in
    your battery, upload code to the arduino, and verify that the motors are
    wired up correctly.

13) Create a power bus for the servos.  I soldered a ~5" wire across four
    male header pins (once for ground, once for power), glued the two sets
    of header pins together, and wrapped heatshrink around them (with slots
    cut to allow the long ends of the header pins to get out).  A less-
    finicky insulation method will probably be better, as long as it's
    secure, and leaves room to plug jumper wires into the signal wires of
    the servo plugs.  Connect this power bus to the +5 and GND screw
    terminals on the outer edge of the speed controller.

14) Plug the servo cables into the power bus (red and brown on the servo
    wire go to +5V and Gnd, respectively), and run jumpers from the Arduino
    to the servo signal wires (orange):
Arduino     Servo
   9   -> Back Left
  10   -> Back Right 
  12   -> Front Left
  13   -> Front Right

15) Press a Wheel Grip onto the outside of each wheel (they should latch
    on fairly tightly), and press the wheel assembly onto the motor drive
    shaft.  The wheels should fit snugly and not feel likely to fall off.
    If they are too loose, adjust the parameters on the Wheel Grips and
    reprint them.  (Note: If a motor fails, you can flip the Wheel Grip
    around (detaching it from the wheel) to allow the wheel to spin freely
    on the drive shaft.)

16) Tighten up all the wires and add some wire management clips (make sure
    the servo wires run along the Front Bars, leaving clearance for the
    wheels to turn inward).  You should now be ready to go!

*/


$fn = 100;

margin = 0.1; // Extra buffer to account for printing error
innermargin = 0.3; // Same, but for small internal holes
wheel_grip_tweak = 0.6; // Extra distance between pegs of wheel gripper, added to width of wheel spokes

X=25.4/10; // Scale factor compared to full-size rover

// 175rpm 12V micro gearmotor: https://www.sparkfun.com/products/12205
// Drawings (ignore the encoder):
// https://www.amazon.com/gp/product/B0195C9GSU
driveMotorRadius = 12/2 +margin/2;
driveMotorFlatRadius = 10/2;
driveMotorSideWidth = 8 +margin;
driveMotorCaseLength = 15.6 +margin;
driveMotorShaftRadius = 3/2 + 0.2; // 2.0 is a hair too big for 3mm shafts // 1.49mm radius is a (difficult) press-fit for 2mm shafts
driveMotorShaftinset = 3 - 2.44 - 0.1; // Final number is press-fit calibration (for 0.1mm layer thickness, Fusion3 F400)
driveMotorGearboxLength = 9;

// SG90 servos: https://www.amazon.com/gp/product/B015H5AVZG
servoCaseLength = 22.4 +innermargin*1.2;
servoCaseWidth = 11.9 +margin;
servoGearboxLength = 14.8 +margin;
servoScrewRadius = 2/2 +margin;
servoMountScrewRadius = 2/2 +margin/2;
servoScrewCenterDistance = 27.5;

servoScrewMountWidth = 5; // exactly 5mm wings on each side for screws
servoMountOffset = 7; // ??? center of servo to screw hole?
servoAxisOffset = 5.1; // Offset from the center of the servo to the rotation axis

// Specs for the single-direction servo horn that came with the servos
servoArmMountRadius = 6.9/2 +margin; // center of rotation to outside of central mount
servoArmLength = 19.6 - servoArmMountRadius +margin*3; // center of rotation to tip of arm
servoArmThickness = 1.5 +innermargin*2;
servoArmNarrowWidth = 4.1 +innermargin;
servoArmWideWidth = 5.4 +innermargin;

// Placeholder printable wheel
tire_width = 9.6*X;
tire_radius = (32/2)*X;
tire_buffer = (6/2.54)*X; // Minimum clearance between tire and frame
tire_curve = 1*X; // Radius of the curve on the sides of the placeholder wheel
spoke_size = 1*X; // Width/thickness of the spokes in the placeholder wheel
hub_rad = driveMotorShaftRadius;    // Size of the opening in the center of the placeholder wheel hub
wall_thickness = 0.5*X; // Thickness of the driving surface of the placeholder wheel


// Dimensions of the LiFePO4 battery
batteryLength = 96.3 + margin;
batteryWidth  = 30.3 + margin;
batteryHeight = 26.6 + margin;



// A few relevant specifications about the rover frame (for the Front Bar module)
frontPlateThickness = 2; // 5 loops at 0.4; 4 loops at 0.5 mm path width
frontPlateWidth = 6*X;
front_section_width = 7.35*X;
bar_width = 2*X;
bar_height = 3*X;
wheelJointDistance = 23*X;
frame_wing_innerwidth = 52.9;
frame_wing_outerwidth = 63.8;
frame_backbone_innerwidth = 13.15;
frame_backbone_outerwidth = 23.9;

// More rover frame specifications (for the Frame module)
max_vehicle_width = 76*X;
end_support_bar_width = 2*X;
wheelSupportWidth = 6*X;
chassis_to_hinge = tire_radius + tire_buffer;
hinge_to_wheel = wheelSupportWidth/2 + tire_buffer;

// Measurements are to center of bars
center_length = 53*X - bar_width; // THIS VALUE CAN BE CHANGED TO FIT THE FRAME ON YOUR PRINT BED (minimum ~97mm)
center_width = 55.45*X - bar_width;
wheelHingeYDist = max_vehicle_width/2 - tire_width - hinge_to_wheel; // Distance from centerline to wheel arm mount point
end_width = wheelHingeYDist*2 - chassis_to_hinge*2 - bar_width; // For LRV-style frame

// Distance from rover center to the front/back wheel mounts
// At closest approach between tire and center chassis section, maintain tire_buffer distance
// LRV was 45.5"
wheelHingeXDist = (center_length+bar_width)/2 + tire_buffer + sqrt( (hinge_to_wheel+tire_width)*(hinge_to_wheel+tire_width) + tire_radius*tire_radius);


// DROK 4-channel speed controller board: https://www.amazon.com/gp/product/B017FZF42G
sc_board_width = 54.4;
sc_board_hole_radius = 3.4/2;
sc_board_hole_inset = 1.25;
scm_peg_offset = sc_board_width/2 - sc_board_hole_radius - sc_board_hole_inset;

// RedBoard Arduino Uno clone
arduino_hole_radius = 3.2/2;
arduino_hole_offset_a =  55/2 - 1.7 - arduino_hole_radius;
arduino_hole_offset_b1 =  55/2 - 6.8 - arduino_hole_radius;
arduino_hole_offset_b2 =  55/2 - 16.9 - arduino_hole_radius;


// Wire strain relief loop.  Fits two Sparkfun 22AWG stranded wires.
module strainReliefLoop(){
    difference(){
            translate([-1,0,0]) cylinder(2,4.5,4.5, center=true);
            cylinder(3,2.1,2.1, center=true);
            translate([-6,0,0])cube(10, center=true);
        }
}

// Steering arm that connects the gearmotor to the servo
module motorGrip(length = 13.6, thickness = 1) { // For motor entirely outside wheel: length = 13.6
    bridge_membrane_thickness = 0.2; // Should produce a single layer with your printing settings (note: S3D apparently can't do a 1-layer bridge)
    motorClipThickness = thickness*1.65;
    driveMotorTotalLength = driveMotorCaseLength + driveMotorGearboxLength;
    mountThickness = thickness*2 + servoArmThickness;
    servoMountBlockLength = servoArmLength + servoArmMountRadius + thickness*2;
    backboneThickness = mountThickness;
    backboneWidth = max(driveMotorSideWidth, servoArmMountRadius*2) + motorClipThickness*2 + 0.3; // Last value is correction factor for non-radial cross-section across clips
    backboneLength = max(length+servoArmMountRadius+thickness*2, driveMotorTotalLength+thickness*2);
    
    ziptie_width = 2.4 + margin;
    ziptie_depth = min(motorClipThickness/2, 0.9); // Zipties are 0.9mm thick
    
    // Motor Grip
    translate([(driveMotorTotalLength/2+thickness)*0,0,-(driveMotorFlatRadius+backboneThickness)])union(){
        // Main backbone/mount plate
        translate([0,0,driveMotorFlatRadius+backboneThickness/2])difference(){
            translate([backboneLength/2-driveMotorTotalLength/2-thickness,0,0]) cube([backboneLength,backboneWidth,backboneThickness], center=true);
            
            // Servo shaft hole
            shaftHoleElongation = servoArmMountRadius/3;
            translate([length-driveMotorTotalLength/2,0,thickness]) cylinder(backboneThickness, servoArmMountRadius, servoArmMountRadius, center=true);
            translate([length-driveMotorTotalLength/2-shaftHoleElongation,0,thickness]) cylinder(backboneThickness, servoArmMountRadius, servoArmMountRadius, center=true);
            translate([length-driveMotorTotalLength/2-shaftHoleElongation/2,0,thickness]) 
                cube([shaftHoleElongation,servoArmMountRadius*2,backboneThickness], center=true);
            
            // Servo screw hole, with thin layer for 
            translate([length-driveMotorTotalLength/2,0,0]) difference(){
                cylinder(backboneThickness+1, servoScrewRadius, servoScrewRadius, center=true);
                translate([0,0,-backboneThickness/2+thickness-0.1]) cube([servoScrewRadius*3, servoScrewRadius*3, bridge_membrane_thickness], center=true);
            }
            // Deep opening for the arm
            translate([length-driveMotorTotalLength/2-servoArmLength/2,0,0]) intersection(){
                cube([servoArmLength,servoArmMountRadius*2,servoArmThickness], center=true);
                rotate([0,90,0])rotate([0,0,45])cylinder(servoArmLength, servoArmNarrowWidth*sqrt(2)/2, servoArmWideWidth*sqrt(2)/2, center=true, $fn=4);
            }
            
            // Ziptie notch
            translate([driveMotorGearboxLength/2,0,0]) difference(){
                cube([ziptie_width+margin,backboneWidth+1,backboneThickness+1], center=true);
                translate([0,0,-ziptie_depth])cube([ziptie_width+1,backboneWidth-ziptie_depth*2,backboneThickness], center=true);
            }
        }
        
        
        
        // Strain relief loop
        translate([backboneLength-driveMotorTotalLength/2-thickness*2-3.5-servoArmMountRadius-servoScrewRadius,0,driveMotorFlatRadius-0.99])
            rotate([0,90,0])strainReliefLoop();
        
        
        // Clips around the side of the motor
        translate([driveMotorGearboxLength/2,0,0])rotate([0,90,0])difference(){
            intersection(){
                cylinder(driveMotorCaseLength/2, driveMotorRadius+motorClipThickness, driveMotorRadius+motorClipThickness, center=true);
                cube([driveMotorFlatRadius*2,driveMotorRadius*3,driveMotorCaseLength], center=true);
            }
            cylinder(driveMotorCaseLength, driveMotorRadius, driveMotorRadius, center=true);
            difference(){
                cylinder(ziptie_width  , driveMotorRadius+motorClipThickness+ziptie_depth, driveMotorRadius+motorClipThickness+ziptie_depth, center=true);
                cylinder(ziptie_width+1, driveMotorRadius+motorClipThickness-ziptie_depth, driveMotorRadius+motorClipThickness-ziptie_depth, center=true);
            }
        }
        
        // Endstops
        translate([driveMotorTotalLength/2+thickness/2,0,driveMotorFlatRadius])
            cube([thickness,backboneWidth,thickness*2], center=true); // inboard endstop
        translate([-driveMotorTotalLength/2-thickness/2,0,driveMotorFlatRadius])
            cube([thickness,backboneWidth,thickness*4], center=true); // outboard endstop
        
    }
    
}
rotate([180,0,0])motorGrip(length = 16.5+tire_width);



  ////////////
 // WHEELS //
////////////

module wheel_printable(){    
    module torus(r1, r2){
        rotate_extrude(convexity = 1)
            translate([r1-r2, 0, 0])
                circle(r = r2, $fn=20);
    }

    module wallBase(radius, widthmod=0){
        union(){
            translate([0,0,tire_width/2-tire_curve]) torus(radius, tire_curve);
            wallHeight = tire_width - tire_curve*2 + widthmod;
            if (widthmod == 0){
            for(i=[1:1:5]) {
                translate([0,0,wallHeight/5*(i-3)])rotate([0,0,9*i])cylinder(wallHeight/5, radius*1.008, radius*1.008, center=true, $fn=20);
            }
            }else {
                cylinder(wallHeight, radius, radius, center=true);
            }
            cylinder(tire_width + widthmod, radius-tire_curve, radius-tire_curve, center=true);
            translate([0,0,-tire_width/2+tire_curve]) torus(radius, tire_curve);
        }
    }
    
    
    module wallBase1(radius, widthmod=0){
        union(){
            translate([0,0,tire_width/2-tire_curve]) torus(radius, tire_curve);
            cylinder(tire_width - tire_curve*2 + widthmod, radius, radius, center=true);
            cylinder(tire_width + widthmod, radius-tire_curve, radius-tire_curve, center=true);
            translate([0,0,-tire_width/2+tire_curve]) torus(radius, tire_curve);
        }
    }

    module wall(){
        difference(){
            wallBase(tire_radius);
            wallBase(tire_radius-wall_thickness, 1);
        }
    }
    
    // Full wheel assembly
    wall();
    intersection(){ // Spokes and hub
        translate([0,0,(-tire_width+spoke_size)/2])difference(){
            union(){
                for(i=[0:45:179]) rotate([0,0,i]) cube([spoke_size,tire_radius*2,spoke_size], center=true);
                cylinder(spoke_size, hub_rad+spoke_size, hub_rad+spoke_size, center=true);
            }
            cylinder(spoke_size+1, hub_rad, hub_rad, center=true);
        }
        wallBase(tire_radius);
    }
}

module pressfittest(count = 6, thickness=spoke_size){
    for(i=[0:1:(count-1)]) translate([11,0,0]*i) difference(){
        cylinder(thickness, driveMotorShaftRadius+spoke_size+0.0*i, driveMotorShaftRadius+spoke_size+0.0*i, center=true);
        difference(){
            cylinder(thickness+1, driveMotorShaftRadius+0.0*i, driveMotorShaftRadius+0.0*i, center=true);
            translate([driveMotorShaftRadius+0.05*i,0,0])cube([driveMotorShaftinset*3,driveMotorShaftRadius*3,thickness+2], center=true);
        }
    }
}

// Part that clips into long-printing wheel and interfaces with the motor shaft.
use <threads.scad>
module wheel_gripper_setscrew(){    
    pegHeight = spoke_size*1.1;
    pegRadius = spoke_size*2/3;
    peg_center_center = 2 * (spoke_size/2 + pegRadius + wheel_grip_tweak/2);
    grip_width = peg_center_center + pegRadius*2;
    
    pegOffset = (peg_center_center/2)/sin(22.5);
    //pegOffset = 8.75;
    //echo(pegOffset);
    
    // Full wheel assembly
    translate([0,0,(-tire_width+spoke_size)/2]) union(){
        for(i=[67.5,112.5,-67.5,-112.5]) rotate([0,0,i]) translate([pegOffset,0,0])
            cylinder(pegHeight,pegRadius,pegRadius, center=true);
        translate([0,0,-pegHeight/2-spoke_size])difference(){
            intersection(){
                cylinder(spoke_size*2,pegOffset+pegRadius,pegOffset+pegRadius, center=true);
                cube([grip_width,(pegOffset+pegRadius)*3,10], center=true);
            }
            difference(){
                cylinder(spoke_size*2+1, driveMotorShaftRadius, driveMotorShaftRadius, center=true);
                translate([driveMotorShaftRadius,0,0])cube([driveMotorShaftinset*3,driveMotorShaftRadius*3,spoke_size*2+2], center=true);
            }
            translate([0,0,-0.5])rotate([0,90,0])metric_thread (diameter=3+0.1, pitch=0.45, length=grip_width/2+1, internal=false); // M2.5 x .45
        }
    }
    
    
}
wheel_gripper_setscrew();

// Spacer 
module wheel_bushing(thickness=1){
    pressfittest(1, thickness);
}



  /////////////////////////////////
 // FRONT BEAM AND SERVO MOUNTS //
/////////////////////////////////


module frontBeam(){
    
    // Servo mount
    module servoMount(){
        translate([0,servoAxisOffset,0])difference() {
            union(){
                cube([frontPlateThickness,servoCaseLength+servoScrewMountWidth*2,frontPlateWidth], center=true);
                translate([-servoMountOffset/2,-servoCaseLength/2-servoScrewMountWidth/2,0])cube([servoMountOffset,servoScrewMountWidth,frontPlateWidth], center=true);
                translate([-servoMountOffset/2, servoCaseLength/2+servoScrewMountWidth/2,0])cube([servoMountOffset,servoScrewMountWidth,frontPlateWidth], center=true);
            
            }
            translate([0, -servoCaseLength/2+servoGearboxLength/2,0]) cube([servoCaseLength,servoGearboxLength,servoCaseWidth], center=true);
            translate([0, servoScrewCenterDistance/2,0])rotate([0,90,0])
                cylinder(servoCaseLength, servoMountScrewRadius,servoMountScrewRadius, center=true);
            translate([0,-servoScrewCenterDistance/2,0])rotate([0,90,0])
                cylinder(servoCaseLength, servoMountScrewRadius,servoMountScrewRadius, center=true);
        }
    }
    
    
    // Frame connection
    module frameConnection(){
    difference(){
        holeWidth = bar_width+innermargin*1.25;
        union(){
            translate([(bar_height+innermargin+frontPlateThickness)/2, front_section_width/2,0])cube([bar_height+innermargin+frontPlateThickness*2, holeWidth+frontPlateThickness*2, frontPlateWidth], center=true);
            translate([(bar_height+innermargin+frontPlateThickness)/2,-front_section_width/2,0])cube([bar_height+innermargin+frontPlateThickness*2, holeWidth+frontPlateThickness*2, frontPlateWidth], center=true);
        }
        
        translate([(bar_height+innermargin+frontPlateThickness)/2, front_section_width/2,frontPlateWidth/2-margin])
            cube([bar_height+innermargin, holeWidth, frontPlateWidth], center=true);
        translate([(bar_height+innermargin+frontPlateThickness)/2,-front_section_width/2,frontPlateWidth/2-margin])
            cube([bar_height+innermargin, holeWidth, frontPlateWidth], center=true);
    }
    }
    
    
    frameConnection();
    
    servoMountLength = servoCaseLength + servoScrewMountWidth - (servoCaseLength/2-servoAxisOffset);
    
    // Top plate
    cube([frontPlateThickness,(wheelJointDistance-servoMountLength)*2+0.01,frontPlateWidth], center=true);
    translate([0,-wheelJointDistance,0])servoMount();
    translate([0, wheelJointDistance,0])rotate([180,0,0])servoMount();
    
    
    
    // Bottom plate
    translate([(bar_height+innermargin+frontPlateThickness), 0,0])
        cube([frontPlateThickness,(wheelJointDistance-servoMountLength)*2,frontPlateWidth], center=true);
    
    // Strain relief loops
    translate([(bar_height+innermargin+frontPlateThickness*2), front_section_width/2,0])
            rotate([90,0,0])strainReliefLoop();
    translate([(bar_height+innermargin+frontPlateThickness*2),-front_section_width/2,0])
            rotate([90,0,0])strainReliefLoop();
    
}




rpi_screw_mount_r = 5; //1.35 + frontPlateThickness;
rpi_case_hole_to_hole = 9.2;
rpi_case_mount_center_to_case_bottom = 22.3;
rpi_case_to_bar = 11; // Offset between edge of case and top of endbar, to allow space for servos
rpi_case_width = 61.2;
rpi_case_side_offset = (82-rpi_case_width)/2; // Horizontal offset to get camera as close to center as possible

rpi_mount_slat_half_distance = 10; // Offset from mounting holes to slide-on tab
rpi_mount_slat_width = 5;
rpi_mount_slat_thick = 3;
rpi_mount_slat_length = 35;

module frontBeam_With_RPi_Mount(){
    rotate([180,0,0])frontBeam();
    
    // Slats to slide Raspberry Pi Adaptor onto
    for (i = [-1,1])
        translate([0,i*rpi_mount_slat_half_distance-rpi_case_side_offset,0]) union(){
            translate([-rpi_mount_slat_length-rpi_case_to_bar-frontPlateThickness,0,-frontPlateWidth+rpi_mount_slat_thick]/2)
                cube([rpi_mount_slat_length+rpi_case_to_bar, rpi_mount_slat_width, rpi_mount_slat_thick], center=true);
            translate([-rpi_case_to_bar-frontPlateThickness, 0, 0]/2)
                cube([rpi_case_to_bar,frontPlateThickness,frontPlateWidth], center=true);
        }
    
    // Alternate fancy integrated mount arm
    module rpi_mount(){
        mount_plate_width = rpi_screw_mount_r*2;
        bar_to_hole_center = rpi_case_to_bar + rpi_case_mount_center_to_case_bottom;
        difference(){
            union(){
                // Vertical riser for the screw holes
                translate([-bar_to_hole_center/2,0,0])
                    cube([bar_to_hole_center,mount_plate_width,frontPlateThickness], center=true);
                
                // Supports for the underside of the Pi
                translate([-rpi_case_to_bar, -frontPlateThickness+mount_plate_width, frontPlateWidth-frontPlateThickness]/2)
                    cube([rpi_case_to_bar,frontPlateThickness,frontPlateWidth], center=true);
                translate([-rpi_case_to_bar,frontPlateThickness-mount_plate_width,frontPlateWidth-frontPlateThickness]/2)
                    cube([rpi_case_to_bar,frontPlateThickness,frontPlateWidth], center=true);
                
                // Thicker potion for the actual screwholes
                hull() {for (i = [-1,1])
                    translate([-bar_to_hole_center - i*rpi_case_hole_to_hole/2,0,frontPlateThickness/2])
                        cylinder(frontPlateThickness*2,rpi_screw_mount_r, rpi_screw_mount_r, center=true);
                }
            }
            
            // Screw holes
            for (i = [-1,1])
                translate([-bar_to_hole_center - i*rpi_case_hole_to_hole/2, 0, -frontPlateThickness*1.5+1])
                    //metric_thread (diameter=2.0, pitch=0.45, length=frontPlateThickness*2.5, internal=true); // M2 x .45
                    //english_thread (diameter=0.086*1.05, threads_per_inch=56, length=frontPlateThickness*2.5/25.4, internal=true); // 2-56
                    english_thread (diameter=0.1120, threads_per_inch=40, length=frontPlateThickness*2.5/25.4, internal=true); // 4-40
        }
    }
    //translate([-frontPlateThickness,-rpi_case_side_offset,-frontPlateWidth+frontPlateThickness]/2) rpi_mount();
    
    // 
    
}
*frontBeam_With_RPi_Mount();

module RPi_adaptor(){
    wall_t = 1.6;
    slat_hole_w = rpi_mount_slat_width+innermargin*2;
    slat_hole_t = rpi_mount_slat_thick+innermargin*2;
    mount_w = rpi_mount_slat_half_distance*2 + slat_hole_w + wall_t*2;
    mount_t = slat_hole_t + wall_t*2;
    mount_h = rpi_case_mount_center_to_case_bottom + rpi_case_hole_to_hole/2 + rpi_screw_mount_r;
    
    nut_r = 7/2+innermargin; // radius of a superscribed circle, aka corner-to-corner distance
    nut_t = 2.4;
    
    difference(){
        translate([-mount_h/2,0,0]) cube([mount_h,mount_w,mount_t], center=true);
        
        for (i = [-1,1])
        translate([0,i*rpi_mount_slat_half_distance,0]) cube([mount_h*3,slat_hole_w,slat_hole_t], center=true);
        
        
        
        // Screw holes
        for (i = [-1,1])
            translate([-rpi_case_mount_center_to_case_bottom - i*rpi_case_hole_to_hole/2, 0,0])union(){
                translate([0,0, mount_t-nut_t+0.01]/2) cylinder(nut_t, nut_r, nut_r, $fn=6, center=true);
                //metric_thread (diameter=2.0, pitch=0.45, length=frontPlateThickness*2.5, internal=true); // M2 x .45
                //english_thread (diameter=0.086*1.05, threads_per_inch=56, length=frontPlateThickness*2.5/25.4, internal=true); // 2-56
                //translate([0,0, -mount_t*1.2/2])english_thread (diameter=0.1120, threads_per_inch=40, length=mount_t*1.2/25.4, internal=true); // 4-40
                cylinder(mount_t*1.2,1.6,1.6, center=true); // Open hole that should fit a 4-40 or M3 bolt
            }
    }
    
}
*RPi_adaptor();


  ///////////
 // FRAME //
///////////


module frame(){
    // Calculation to allow non-90deg tire to rotate at least tireMaxAngle degrees
    // side_angle of 50 degrees matches LRV max steering angle; Drawing 4.6
    tireBaseAngle = atan(hinge_to_wheel/tire_radius);
    tireBaseDistance = sqrt(tire_radius*tire_radius + hinge_to_wheel*hinge_to_wheel);
    tireInsetDistance = max(0, sin(90 - tireBaseAngle) * tireBaseDistance) + end_support_bar_width/2 + tire_buffer;
    
    // Distance from centerline of vehicle
    endSupportSideOffset = (end_width+bar_width)/2+chassis_to_hinge - tireInsetDistance;
    
    // End bars (lengthwise)
    endSupportLength = wheelHingeXDist - center_length/2; // end of center section to wheel mount distance
    endSupportOffset = wheelHingeXDist - endSupportLength/2;
    translate([-endSupportOffset,-endSupportSideOffset,0]) cube([endSupportLength,end_support_bar_width,bar_height], center=true);
    translate([-endSupportOffset, endSupportSideOffset,0]) cube([endSupportLength,end_support_bar_width,bar_height], center=true);
    translate([endSupportOffset, endSupportSideOffset,0]) cube([endSupportLength,end_support_bar_width,bar_height], center=true);
    translate([endSupportOffset,-endSupportSideOffset,0]) cube([endSupportLength,end_support_bar_width,bar_height], center=true);
    
    
    // Center section endbars
    wingBarLength = center_width/2-endSupportSideOffset;
    translate([-center_length/2,wingBarLength/2+endSupportSideOffset,0])cube([bar_width,wingBarLength,bar_height], center=true);
    translate([ center_length/2,wingBarLength/2+endSupportSideOffset,0])cube([bar_width,wingBarLength,bar_height], center=true);
    translate([-center_length/2,-wingBarLength/2-endSupportSideOffset,0])cube([bar_width,wingBarLength,bar_height], center=true);
    translate([ center_length/2,-wingBarLength/2-endSupportSideOffset,0])cube([bar_width,wingBarLength,bar_height], center=true);
    
    // Center section sidebars
    translate([0,-center_width/2,0])cube([center_length+bar_width,bar_width,bar_height], center=true);
    translate([0, center_width/2,0])cube([center_length+bar_width,bar_width,bar_height], center=true);
    
    ////////////
    // Extra stiffener plate for the backbone/wings
    // Bounding box size for the wheel turning arc, with margin
    wheelArcDiameter = sqrt((hinge_to_wheel+tire_width)*(hinge_to_wheel+tire_width)+tire_radius*tire_radius)*2+tire_buffer*2;
    
    // Get a point along the edge of the cutout cube
    xt1 = wheelHingeXDist - wheelArcDiameter/2*sqrt(2)/2;
    yt1 = wheelHingeYDist - wheelArcDiameter/2*sqrt(2)/2;
    
    // Get the intersection points of the cutout cubes and the outer edges of the frame
    stiffenPlateXEdge = xt1 + yt1-(endSupportSideOffset+bar_width/2);
    stiffenPlateYEdge = yt1 + xt1-(center_length/2+bar_width/2);
    translate([0,0,-bar_height/2+X/8]) difference() {
        cutoutYIntersection = 1;// point where the cutout hits the outside of the center section
        cutoutXIntersection = 1;// point where the cutout hits the outside of the endbars
        cube([stiffenPlateXEdge*2,stiffenPlateYEdge*2,X/4], center=true);
        translate([ wheelHingeXDist, wheelHingeYDist,0]) rotate([  0,0,45]) cube([wheelArcDiameter,wheelArcDiameter,X], center=true);
        translate([-wheelHingeXDist, wheelHingeYDist,0]) rotate([  0,0,45]) cube([wheelArcDiameter,wheelArcDiameter,X], center=true);
        translate([ wheelHingeXDist,-wheelHingeYDist,0]) rotate([  0,0,45]) cube([wheelArcDiameter,wheelArcDiameter,X], center=true);
        translate([-wheelHingeXDist,-wheelHingeYDist,0]) rotate([  0,0,45]) cube([wheelArcDiameter,wheelArcDiameter,X], center=true);
        
    }
    
    // Main backbone (lengthwise)
    translate([0, endSupportSideOffset,0]) cube([center_length+bar_width,end_support_bar_width,bar_height], center=true);
    translate([0,-endSupportSideOffset,0]) cube([center_length+bar_width,end_support_bar_width,bar_height], center=true);
    // Backbone crossbrace(s)
    translate([0,0,0]) cube([bar_width,endSupportSideOffset*2,bar_height], center=true);
    //translate([stiffenPlateXEdge-bar_width/2,0,0]) cube([bar_width,endSupportSideOffset*2,bar_height], center=true);
    //translate([-stiffenPlateXEdge+bar_width/2,0,0]) cube([bar_width,endSupportSideOffset*2,bar_height], center=true);
}

  //////////////////////////
 // CIRCUIT BOARD MOUNTS //
//////////////////////////


module speedControlMount(peg_offset_1, peg_offset_2, hole_radius){
    peg_radius = hole_radius - margin/2;
    scm_thickness = 2;
    scm_mount_depth = 6;
    scm_length =  frame_wing_outerwidth + scm_thickness*2;
    scm_clip_margin = -0.1;
    
    cube([scm_thickness, scm_length, peg_radius*2], center=true);
    
    translate([-scm_thickness, peg_offset_1,0]) rotate([0,90,0])
        cylinder(scm_thickness*3, peg_radius, peg_radius, center=true);
    translate([-scm_thickness,-peg_offset_2,0]) rotate([0,90,0])
        cylinder(scm_thickness*3, peg_radius, peg_radius, center=true);
    
    translate([scm_mount_depth/2, frame_wing_outerwidth/2+scm_thickness/2+scm_clip_margin,0])
        cube([scm_mount_depth+scm_thickness, scm_thickness, peg_radius*2], center=true);
    translate([scm_mount_depth/2,-frame_wing_outerwidth/2-scm_thickness/2-scm_clip_margin,0])
        cube([scm_mount_depth+scm_thickness, scm_thickness, peg_radius*2], center=true);
    
    translate([scm_mount_depth/2, frame_wing_innerwidth/2-scm_thickness/2-scm_clip_margin,0])
        cube([scm_mount_depth+scm_thickness, scm_thickness, peg_radius*2], center=true);
    translate([scm_mount_depth/2,-frame_wing_innerwidth/2+scm_thickness/2+scm_clip_margin,0])
        cube([scm_mount_depth+scm_thickness, scm_thickness, peg_radius*2], center=true);
    
}


module wireManagementClip(batteryWidth = 0, batteryAnchorHeight = 0){
    thickness = 2;
    scm_mount_depth = 6;
    length =  max(batteryWidth, frame_backbone_outerwidth) + thickness*2;
    scm_clip_margin = -0.1;
    
    cube([thickness, length, thickness], center=true);
    
    
    translate([scm_mount_depth/2, frame_backbone_outerwidth/2+thickness/2+scm_clip_margin,0])
        cube([scm_mount_depth+thickness, thickness, thickness], center=true);
    translate([scm_mount_depth/2,-frame_backbone_outerwidth/2-thickness/2-scm_clip_margin,0])
        cube([scm_mount_depth+thickness, thickness, thickness], center=true);
    
    translate([scm_mount_depth/2, frame_backbone_innerwidth/2-thickness/2-scm_clip_margin,0])
        cube([scm_mount_depth+thickness, thickness, thickness], center=true);
    translate([scm_mount_depth/2,-frame_backbone_innerwidth/2+thickness/2+scm_clip_margin,0])
        cube([scm_mount_depth+thickness, thickness, thickness], center=true);
    
    // Optional battery-restraint posts
    if (batteryWidth > 0) {
        translate([-batteryAnchorHeight/2, batteryWidth/2+thickness/2,0])
            cube([batteryAnchorHeight+thickness, thickness, thickness], center=true);
        translate([-batteryAnchorHeight/2,-batteryWidth/2-thickness/2,0])
            cube([batteryAnchorHeight+thickness, thickness, thickness], center=true);
    }
}
wireManagementClip();

  //////////////////////
 // MODULE SELECTION //
//////////////////////

speedControlMount(scm_peg_offset, scm_peg_offset, sc_board_hole_radius); // Print 2
speedControlMount(arduino_hole_offset_a, arduino_hole_offset_a, arduino_hole_radius); // Print 1
speedControlMount(arduino_hole_offset_b1, arduino_hole_offset_b2, arduino_hole_radius); // Print 1
wireManagementClip(); // Print at least 4

wireManagementClip(14, 30); // Phone battery pack (print 2)
!wireManagementClip(27, 20); // LiFe Battery (print 2)


rotate([180,0,0])motorGrip(length = 16.5+tire_width); // Print 4
frontBeam(); // Print 1
frontBeam_With_RPi_Mount(); // Print 1
wheel_printable(); // Print 4
rotate([0,90,0]) wheel_gripper_setscrew(); // Print 4 (suggested layer thickness: 0.1mm)
wheel_bushing(); // Print 4
frame(); // Print 1
!RPi_adaptor(); // Print 1
