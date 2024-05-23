// This file is useful for visualizing the rover pose relative to turning centers
// It also contains code for visualizing alternate rover geometries,
// based on the desired maximum wheel turning angles.
// You can print the parts, too, for a fun little desk toy.  Pit the parts together with cut-up bits of paperclip.

X = 25.4/20; // scaling factor for the frame (original is inches)

// Vehicle geometry constraints
max_vehicle_width = 71*X; // 76*X;
maxOutwardAngle   = 90 ; // e.g. max amount the front-left wheel can turn from forward to left
maxInwardAngle    =  0; // e.g. max amount the front-left wheel can turn from forward to right (minus 90 degrees)

// The LRV had a minimum turning radius of ~72.75" (1.85m), with a wheel angle of 50deg
// With our longer hinge-to-wheel distance and associated frame changes, we get that radius with a max wheel angle of 45deg

// Direction to point the wheels (see also "steeringMode" below)
wheelAngle   = 0;
turnCenterX  = 0; turnCenterY  = 72.75*X; // LRV Min Radius
//turnCenterX  = 120; turnCenterY  = 0; // Circle-strafe
//turnCenterX  = 90; turnCenterY  = 72.75*X; // Arbitrary
//turnCenterX  = 5; turnCenterY  = -20; // Perfect panorama
//turnCenterX  = 65; turnCenterY  = 0; // Perfect rovercam panorama

steeringMode = 0; // 0: wheels turn about the point defined above;   1: Wheels turn in strafing mode, at the angle defined above
//wheelAngle=$t*120-60; // Animation
//turnCenterX = -10; turnCenterY = 20; // For panoramas by the copilot
//turnCenterX = 0; turnCenterY = $t*300-150 ; // animation

//steeringMode = 1;  wheelAngle   = 45;

backboneFrame = true; // If true, make frame using full-length backbone-style bars

// Display options
showWheelSweep              = false; // Show a transparent shape over the area that the wheel can occupy
showTurnPointExclusionZones = false; // Draw thick red lines in regions where the center of rotation cannot be  for normal and strafing turns
showFullExclusionZones      = true; // Draw wedges showing the full 2D exclusion zones
showWheelDirectionLine      = true; // Draw light purple lines perpendicular to each wheel
showCurrentTurnCenter       = true; // Places a magenta cylinder at the center of rotation
useSimpleWheel              = true; // Using non-simple wheels takes much longer to render
animateTurnPoint            = true; // Center the turning point in the grid and rotate the model about that point (requires checking "Animate" in the View menu)


//dimensionDisplayUnits="meters"; dimensionDisplayMultiplier=0.0254;
dimensionDisplayUnits="inches"; dimensionDisplayMultiplier=1;

// Color definitions
// https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Transformations#color
exclusionColor      = "Red";
wheelDirectionColor = "Thistle";
turnCenterColor     = "Magenta";



//////////////////////////
// PRINTING ADJUSTMENTS //
//////////////////////////

// Radius of pins used for articulation (in this case, ordinary paperclips)
pinRadius = 1/2;
// The above scaled appropriately for BFB 3DTouch, with hole oriented horizontally/vertically
pinRadius_h = 2.0/2; 
pinRadius_v = 2.6/2; 

c_thickness = 1; // Thickness of the plates holding the wheel arms to the main body
printingBuffer = 1.4; // Extra clearance needed for the wheel arm "C"s to clear the wheel supports on the work printer


///////////////////////////////////////
//       Dimensional Variables       //
//           DO NOT TOUCH!           //
///////////////////////////////////////

$fn=200;

Y = X; // Scaling factor for the bars of the frame
bar_width = 2*Y;
bar_height = 3*Y;
end_support_bar_width = 2*Y;

tire_width = 9.6*X;
tire_radius = (32/2)*X;
tire_buffer = (5/2.54)*X; // Minimum clearance between tire and frame

wheelSupportWidth = 6*X;//bar_width * 2;
wheelSupportHeight = 6*Y;
chassis_to_hinge = tire_radius + tire_buffer;
wheelSupportLength = chassis_to_hinge + wheelSupportWidth/2;
hinge_to_wheel = wheelSupportWidth/2 + tire_buffer;

// Verical offset of wheels
wheel_mount_offset=1*X+bar_height/2;

// Measurements are to center of bars
center_length = 53*X - bar_width;
center_width = 55.45*X - bar_width;
front_length = 29*X - bar_width/2; // LRV = 29"
back_length = 29*X - bar_width/2; // LRV = 27.5"
wheelHingeYDist = max_vehicle_width/2 - tire_width - hinge_to_wheel; // Distance from centerline to wheel arm mount point
end_width = wheelHingeYDist*2 - chassis_to_hinge*2 - bar_width; // For LRV-style frame

// Distance from rover center to the front/back wheel mounts
// At closest approach between tire and center chassis section, maintain tire_buffer distance
// LRV was 45.5"
wheelHingeXDist = (center_length+bar_width)/2 + tire_buffer + sqrt( (hinge_to_wheel+tire_width)*(hinge_to_wheel+tire_width) + tire_radius*tire_radius);

// Placeholder printable wheel
tire_curve = 1*X; // Radius of the curve on the sides of the placeholder wheel
spoke_size = 1*X; // Width/thickness of the spokes in the placeholder wheel
hub_rad = 2.35*X;    // Size of the opening in the center of the placeholder wheel hub
wall_thickness = 0.5*X; // Thickness of the driving surface of the placeholder wheel

// Placeholder for steering motor
// http://www.robotmarketplace.com/products/NPC-B81HT.html
motor_length=15*X-wheelSupportWidth; // The -wheelSupportWidth is to allow clearance for the functional hinge
motor_radius=3.75/2*X;




///////////////////////////////////////
//              Modules              //
///////////////////////////////////////

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


module wheel_simple(){
    cylinder(tire_width, tire_radius, tire_radius, center=true);
}

module wheel(){
    if (showWheelDirectionLine) color(wheelDirectionColor) translate([0,0,-wheel_mount_offset])rotate([0,0,0])cylinder(1000,1,1,center=true);
    if(useSimpleWheel)
        wheel_simple();
    else
       wheel_printable();
}

module wheel_sweep(){
    t = [0,0,hinge_to_wheel+tire_width/2];
    translate(t) for(i = [-(90+maxInwardAngle):10:maxOutwardAngle]) rotate([0,180-i,0])
        translate(t) cylinder(tire_width, tire_radius, tire_radius, center=true);
}


module lrv_wheel(){
    translate([0,wheel_dist+tire_width/2,0])rotate([90,0,0])difference(){union(){
        rotate_extrude() translate([tire_radius-tire_width/2, 0, 0]) circle(tire_width/2); 
        cylinder(tire_width*0.75,tire_radius-tire_width/2,tire_radius-tire_width/2, center=true);
        
        translate([0,bar_width+0.1,tire_width/2-bar_width])cube([tire_width,bar_width,bar_width*2], center=true);
        translate([0,-bar_width-0.1,tire_width/2-bar_width])cube([tire_width,bar_width,bar_width*2], center=true);
    }
        translate([0,0,tire_width/2-bar_width])cube([tire_width,bar_width,bar_width], center=true);
    }
}


//////////////////////
// WHEEL SWING ARMS //
//////////////////////

module wheel_arm_basic(){
    barlen=hinge_to_wheel + tire_width-spoke_size;
    mountlen = spoke_size + 5*pinRadius_h; // How far the wheel mount protrudes from the end of the bar
    
    difference(){
        union(){
            // Main bar
            translate([0,(barlen)/2,0])cube([wheelSupportWidth,barlen,wheelSupportHeight+c_thickness*2+printingBuffer], center=true);
            // Fancy-looking rounded end
            cylinder(wheelSupportHeight+c_thickness*2+printingBuffer, wheelSupportWidth/2, wheelSupportWidth/2, center=true);
            // Wheel mount (extends 2.5*spoke_size
            translate([0,barlen/2+mountlen,-wheel_mount_offset])rotate([90,0,0])cylinder(barlen, hub_rad-0.6, hub_rad-0.6, center=true);
        }
        
        // C opening will be wheelSupportWidth deep
        cube([wheelSupportWidth*2,wheelSupportWidth*2,wheelSupportHeight+printingBuffer], center=true);
        
        // Hinge pin
        cylinder(bar_height*2,pinRadius_h,pinRadius_h, center=true);
        
        // Wheel-anchoring pin
        translate([0,barlen+spoke_size+pinRadius_h*2.5,0])cylinder(wheelSupportHeight*2,pinRadius_h,pinRadius_h, center=true);
    
    }
}


module wheel_arm(){
    barlen=hinge_to_wheel + tire_width-spoke_size;
    barplusc = wheelSupportHeight+c_thickness*2+printingBuffer;
    mountlen = spoke_size + 5*pinRadius_h; // How far the wheel mount protrudes from the end of the bar
    
    wheel_inset = 3*X;
    gearboxlen = 4.5*X; // probably an underestimate
    motorlen = 7.5*X;
    
    motoroffset=wheel_mount_offset+2*X;
    difference(){
        union(){
            // Angled bar
            translate([0,(barlen)/2,0])difference(){
            cube([wheelSupportWidth,barlen,barplusc], center=true);
            translate([0,wheelSupportWidth+c_thickness+printingBuffer,0])rotate([-45,0,0])cube([wheelSupportWidth+printingBuffer,barlen*2,barplusc], center=true);
            // Wheel inset cutoff
            translate([0,barlen-wheel_inset,0])cube([wheelSupportWidth+printingBuffer,barlen,barlen], center=true);
                
            }
            // Fancy-looking rounded end
            cylinder(wheelSupportHeight+c_thickness*2+printingBuffer, wheelSupportWidth/2, wheelSupportWidth/2, center=true);
            
            translate([0,0,-wheel_mount_offset])union(){
                // wheel stop
                translate([0,barlen-c_thickness/2,0])cube([wheelSupportWidth,c_thickness,barplusc], center=true);
                // Wheel mount (extends 2.5*spoke_size
                translate([0,barlen/2+mountlen,0])rotate([90,0,0])cylinder(barlen, hub_rad-0.6, hub_rad-0.6, center=true);
            }
                // Motor assembly
            translate([0,0,-motoroffset])union(){
                // Motor gearbox
                translate([0,barlen-gearboxlen/2-wheel_inset,0])cube([wheelSupportWidth,gearboxlen,barplusc], center=true);
                // Motor cylinder
                translate([0,barlen-gearboxlen-motorlen/2-wheel_inset,0])rotate([90,0,0])cylinder(motorlen, motor_radius,motor_radius, center=true);
            }
        }
        
        // C opening will be wheelSupportWidth deep
        cube([wheelSupportWidth*2,wheelSupportWidth*2,wheelSupportHeight+printingBuffer], center=true);
        
        // Hinge pin
        cylinder(bar_height*5,pinRadius_h,pinRadius_h, center=true);
        
        // Wheel-anchoring pin
        translate([0,barlen+spoke_size+pinRadius_h*2.5,0])cylinder(wheelSupportHeight*4,pinRadius_h,pinRadius_h, center=true);
    }
}

// Place the wheels and wheel arms at the correct locations and angles
module wheel_demo(turnCenterX = 0, turnCenterY = 0){
    // Optionally display the center of rotation
    if (steeringMode == 0 && showCurrentTurnCenter) {
        color(turnCenterColor) translate([turnCenterX,turnCenterY,-wheel_mount_offset])cylinder( (tire_radius+wheel_mount_offset)*2,2*X,2*X, center=true);
    }
    
    // Intermediate value for calculating the wheel angles
    distYLeft  = turnCenterY-wheelHingeYDist;
    distYRight = turnCenterY+wheelHingeYDist;
    distXFront = -turnCenterX-wheelHingeXDist;
    distXBack  = -turnCenterX+wheelHingeXDist;
    
    // Calculate the angle for each wheel.  Use only the angle appropriate to the steering mode.
    // If the wheel would run into the frame on the inside, flip it to the other side
    angleFL = ( atan(distXBack /distYLeft)         - ((atan(distXBack /distYLeft )> maxOutwardAngle)?180:0) )*(1-steeringMode) + (   -wheelAngle)*steeringMode;
    angleBL = ( atan(distXFront/distYLeft)         + ((atan(distXFront/distYLeft )<-maxOutwardAngle)?180:0) )*(1-steeringMode) + (   -wheelAngle)*steeringMode;
    angleFR = ( atan(distXBack /distYRight) + 180  + ((atan(distXBack /distYRight)<-maxOutwardAngle)?180:0) )*(1-steeringMode) + (180-wheelAngle)*steeringMode;
    angleBR = ( atan(distXFront/distYRight) + 180  - ((atan(distXFront/distYRight)> maxOutwardAngle)?180:0) )*(1-steeringMode) + (180-wheelAngle)*steeringMode;
    
    // Mark all wheels with frame collisions
    module errorX(location){
        translate(location)union(){
            color(exclusionColor)rotate([ 90,0, 45])cylinder(wheelHingeXDist*1,3*X,3*X,center=true);
            color("Black")rotate([ 90,0, 45])scale([1.3,0.5,1.05])cylinder(wheelHingeXDist*1,3*X,3*X,center=true);
            color(exclusionColor)rotate([ 90,0,-45])cylinder(wheelHingeXDist*1,3*X,3*X,center=true);
            color("Black")rotate([ 90,0,-45])scale([1.3,0.5,1.05])cylinder(wheelHingeXDist*1,3*X,3*X,center=true);
        }
    }
    if (angleFL < -90-maxInwardAngle ) errorX([ wheelHingeXDist, wheelHingeYDist, 0]);
    if (angleBL > 90+maxInwardAngle ) errorX([-wheelHingeXDist, wheelHingeYDist, 0]);
    if ((angleBR-180) < (-90-maxInwardAngle) ) errorX([-wheelHingeXDist,-wheelHingeYDist, 0]);
    if ((angleFR-180) > 90+maxInwardAngle ) errorX([ wheelHingeXDist,-wheelHingeYDist, 0]);
    
    // Place the wheel arms and wheels
    translate([-wheelHingeXDist, wheelHingeYDist, 0]) rotate([0,0,angleBL]) union(){
        wheel_arm();
        translate([0,hinge_to_wheel+tire_width/2,-wheel_mount_offset]) rotate([90,0,0])wheel();
        if(showWheelSweep) %translate([0,0,-wheel_mount_offset]) rotate([0,0,-angleBL]) translate([0,hinge_to_wheel+tire_width/2,0]) rotate([90,0,0]) wheel_sweep();
    }
    translate([ wheelHingeXDist, wheelHingeYDist, 0]) rotate([0,0,angleFL])union(){
        wheel_arm();
        translate([0,hinge_to_wheel+tire_width/2,-wheel_mount_offset]) rotate([90,0,0])wheel();
        if(showWheelSweep) %translate([0,0,-wheel_mount_offset]) rotate([0,180,-angleFL]) translate([0,hinge_to_wheel+tire_width/2,0]) rotate([90,0,0]) wheel_sweep();
    }
    translate([-wheelHingeXDist, -wheelHingeYDist, 0]) rotate([0,0,angleBR])union(){
        wheel_arm();
        translate([0,hinge_to_wheel+tire_width/2,-wheel_mount_offset]) rotate([90,0,0])wheel();
        if(showWheelSweep) %translate([0,0,-wheel_mount_offset]) rotate([0,180,180-angleBR]) translate([0,hinge_to_wheel+tire_width/2,0]) rotate([90,0,0]) wheel_sweep();
    }
    translate([ wheelHingeXDist, -wheelHingeYDist, 0]) rotate([0,0,angleFR])union(){
        wheel_arm();
        translate([0,hinge_to_wheel+tire_width/2,-wheel_mount_offset]) rotate([90,0,0])wheel();
        if(showWheelSweep) %translate([0,0,-wheel_mount_offset]) rotate([0,0,180-angleFR]) translate([0,hinge_to_wheel+tire_width/2,0]) rotate([90,0,0]) wheel_sweep();
    }
}


///////////
// FRAME //
///////////

module wheel_support(){
    difference(){
        translate([0,wheelSupportLength/2-0.01,0]) cube([wheelSupportWidth,wheelSupportLength,wheelSupportHeight], center=true);
        translate([0,chassis_to_hinge,0])cylinder(wheelSupportHeight*2,pinRadius_v,pinRadius_v, center=true);
    }
    // Hard stop on rotation, with aligned motor
    translate([0,chassis_to_hinge-wheelSupportWidth-0.4,(wheelSupportHeight+c_thickness+printingBuffer/2)/2]) cube([wheelSupportWidth,wheelSupportWidth,c_thickness+printingBuffer/2], center=true);
    translate([0,chassis_to_hinge-wheelSupportWidth/2-0.4-motor_length/2,wheelSupportHeight/2+motor_radius]) rotate([90,0,0]) cylinder(motor_length,motor_radius,motor_radius, center=true);
}

// LRV-style frame
module frame(){
    // Wheel fixed arms
    translate([-wheelHingeXDist, (end_width+bar_width)/2,0])rotate([  0,0,0])wheel_support();
    translate([-wheelHingeXDist,-(end_width+bar_width)/2,0])rotate([  0,0,180])wheel_support();
    translate([wheelHingeXDist,-(end_width+bar_width)/2,0])rotate([  0,0,180])wheel_support();
    translate([wheelHingeXDist, (end_width+bar_width)/2,0])rotate([  0,0,  0])wheel_support();
    
    // Center section end bars
    centerSideWidth = (center_width+bar_width)/2 - end_width/2;
    translate([-center_length/2, (centerSideWidth+end_width)/2,0])cube([bar_width,centerSideWidth,bar_height], center=true);
    translate([ center_length/2, (centerSideWidth+end_width)/2,0])cube([bar_width,centerSideWidth,bar_height], center=true);
    
    translate([-center_length/2,-(centerSideWidth+end_width)/2,0])cube([bar_width,centerSideWidth,bar_height], center=true);
    translate([ center_length/2,-(centerSideWidth+end_width)/2,0])cube([bar_width,centerSideWidth,bar_height], center=true);
    
    // Center section sidebars
    translate([0,-center_width/2,0])cube([center_length+bar_width,bar_width,bar_height], center=true);
    translate([0, center_width/2,0])cube([center_length+bar_width,bar_width,bar_height], center=true);
    
    // End bars (lengthwise)
    //translate([-(center_length+front_length)/2-bar_width,-end_width/2,0])cube([front_length+bar_width,bar_width,bar_height], center=true);
    //translate([-(center_length+front_length)/2-bar_width, end_width/2,0])cube([front_length+bar_width,bar_width,bar_height], center=true);
    //translate([(center_length+front_length)/2+bar_width,-end_width/2,0])cube([front_length+bar_width,bar_width,bar_height], center=true);
    //translate([(center_length+front_length)/2+bar_width, end_width/2,0])cube([front_length+bar_width,bar_width,bar_height], center=true);
    
    // End bars (lengthwise, full length)
    translate([0,-end_width/2,0])cube([front_length+center_length+back_length+bar_width*3,bar_width,bar_height], center=true);
    translate([0, end_width/2,0])cube([front_length+center_length+back_length+bar_width*3,bar_width,bar_height], center=true);
    
    
    translate([-center_length/2-front_length-bar_width,0,0])cube([bar_width,end_width+bar_width,bar_height], center=true);
    //translate([-center_length/2-front_length/2-bar_width/2,0,0])cube([bar_width,end_width+bar_width,bar_height], center=true);
    translate([ center_length/2+back_length+bar_width,0,0])cube([bar_width,end_width+bar_width,bar_height], center=true);
    
    // Center crossbars (offset to make continuity more clear)
  //  translate([ center_length/2-bar_width,0,0])cube([bar_width,end_width+bar_width,bar_height], center=true);
  //  translate([-center_length/2+bar_width,0,0])cube([bar_width,end_width+bar_width,bar_height], center=true);
}

// Dynamically-modified frame based on the wheel angles defined at the top of this document
module angle_frame(side_angle=90, front_angle=0){
    if( (side_angle+front_angle) > 90 ) {
        side_angle=90;
        front_angle=0;
    }
    
    
    
    // Calculation to allow non-90deg tire to rotate at least tireMaxAngle degrees
    // side_angle of 50 degrees matches LRV max steering angle; Drawing 4.6
    tireBaseAngle = atan(hinge_to_wheel/tire_radius);
    tireBaseDistance = sqrt(tire_radius*tire_radius + hinge_to_wheel*hinge_to_wheel);
    tireInsetDistance = max(0, sin(side_angle - tireBaseAngle) * tireBaseDistance) + end_support_bar_width/2 + tire_buffer;
    
    // Distance from centerline of vehicle
    endSupportSideOffset = (end_width+bar_width)/2+chassis_to_hinge - tireInsetDistance;
    
    
    // End bars (lengthwise)
    endSupportLength = wheelHingeXDist - center_length/2 - tireInsetDistance*sin(front_angle); // end of center section to wheel mount distance
    endSupportOffset = wheelHingeXDist - endSupportLength/2 - tireInsetDistance*sin(front_angle);
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
    
    
    
    
    // End bar (cross-wise) needs to be (distance between wheel supports with no angle) + 2 times the distance from the frame bars 
    endBarLength = (end_width+bar_width) + 2*(chassis_to_hinge-sqrt(wheelSupportWidth*wheelSupportWidth/4+chassis_to_hinge*chassis_to_hinge)*cos(front_angle+atan(wheelSupportWidth/2/chassis_to_hinge) ));
    endBarOffset=wheelHingeXDist-chassis_to_hinge*sin(front_angle);
    
    // Endbars ([angled] wheelSupports + cross-wise bar)
    translate([-wheelHingeXDist, (end_width+bar_width)/2+chassis_to_hinge,0])rotate([  0,0,front_angle])translate([0,-chassis_to_hinge,0])wheel_support();
    translate([-endBarOffset, 0,0]) cube([wheelSupportWidth,endBarLength,wheelSupportHeight], center=true);
    translate([-wheelHingeXDist,-(end_width+bar_width)/2-chassis_to_hinge,0])rotate([  0,0,180-front_angle])translate([0,-chassis_to_hinge,0])wheel_support();
    
    translate([wheelHingeXDist,-(end_width+bar_width)/2-chassis_to_hinge,0])rotate([  0,0,180+front_angle])translate([0,-chassis_to_hinge,0])wheel_support();
    translate([endBarOffset, 0,0]) cube([wheelSupportWidth,endBarLength,wheelSupportHeight], center=true);
    translate([wheelHingeXDist, (end_width+bar_width)/2+chassis_to_hinge,0])rotate([  0,0,  -front_angle])translate([0,-chassis_to_hinge,0])wheel_support();
    
    
    // Extra support plate for endbars (only when width < 12"
    color("Orange") if ( !backboneFrame && (endSupportSideOffset*2+bar_width)/X < 12) {
        plateThickness=X/4;
        plateWidth=endSupportSideOffset*2+end_support_bar_width;
        translate([-endBarOffset-wheelSupportWidth/2+plateWidth/2,0, (bar_height+plateThickness)/2]) cube([plateWidth,plateWidth,plateThickness], center=true);
        translate([ endBarOffset+wheelSupportWidth/2-plateWidth/2,0, (bar_height+plateThickness)/2]) cube([plateWidth,plateWidth,plateThickness], center=true);
        translate([-endBarOffset-wheelSupportWidth/2+plateWidth/2,0,-(bar_height-plateThickness)/2]) cube([plateWidth,plateWidth,plateThickness], center=true);
        translate([ endBarOffset+wheelSupportWidth/2-plateWidth/2,0,-(bar_height-plateThickness)/2]) cube([plateWidth,plateWidth,plateThickness], center=true);
    }
    
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
    color("Orange") translate([0,0,-bar_height/2+X/8]) difference() {
        cutoutYIntersection = 1;// point where the cutout hits the outside of the center section
        cutoutXIntersection = 1;// point where the cutout hits the outside of the endbars
        cube([stiffenPlateXEdge*2,stiffenPlateYEdge*2,X/4], center=true);
        translate([ wheelHingeXDist, wheelHingeYDist,0]) rotate([  0,0,45]) cube([wheelArcDiameter,wheelArcDiameter,X], center=true);
        translate([-wheelHingeXDist, wheelHingeYDist,0]) rotate([  0,0,45]) cube([wheelArcDiameter,wheelArcDiameter,X], center=true);
        translate([ wheelHingeXDist,-wheelHingeYDist,0]) rotate([  0,0,45]) cube([wheelArcDiameter,wheelArcDiameter,X], center=true);
        translate([-wheelHingeXDist,-wheelHingeYDist,0]) rotate([  0,0,45]) cube([wheelArcDiameter,wheelArcDiameter,X], center=true);
        
    }
    echo(stiffenPlateXEdge/X);
    echo(stiffenPlateYEdge/X);
    
    
    if (!backboneFrame) {
        // Extra center-section longitudinal bar
        translate([0,0,0])cube([center_length+bar_width,bar_width,bar_height], center=true);
        translate([-center_length/2,0,0])cube([bar_width,endSupportSideOffset*2,bar_height], center=true);
        translate([ center_length/2,0,0])cube([bar_width,endSupportSideOffset*2,bar_height], center=true);
    } else {
        // Main backbone (lengthwise)
        translate([0, endSupportSideOffset,0]) cube([center_length+bar_width,end_support_bar_width,bar_height], center=true);
        translate([0,-endSupportSideOffset,0]) cube([center_length+bar_width,end_support_bar_width,bar_height], center=true);
        // Backbone crossbrace(s)
        translate([0,0,0]) cube([bar_width,endSupportSideOffset*2,bar_height], center=true);
        translate([stiffenPlateXEdge-bar_width/2,0,0]) cube([bar_width,endSupportSideOffset*2,bar_height], center=true);
        translate([-stiffenPlateXEdge+bar_width/2,0,0]) cube([bar_width,endSupportSideOffset*2,bar_height], center=true);
        
    }
    
    // Additional support arms if the wheels can't rotate a full 180
    if ( (side_angle+front_angle) < 90 ) {
        echo(endSupportSideOffset=(endSupportSideOffset),"inches");
        secondaryArmLength = (tireInsetDistance) / sin(side_angle);
        translate([-wheelHingeXDist, (end_width+bar_width)/2+chassis_to_hinge,0]) rotate([  0,0, 90-side_angle]) translate([0,-secondaryArmLength/2 - pinRadius_v,0])cube([bar_width,secondaryArmLength - pinRadius_v,bar_height], center=true);
        translate([-wheelHingeXDist,-(end_width+bar_width)/2-chassis_to_hinge,0]) rotate([  0,0, 90+side_angle]) translate([0,-secondaryArmLength/2 - pinRadius_v,0]) cube([bar_width,secondaryArmLength - pinRadius_v,bar_height], center=true);
        translate([ wheelHingeXDist,-(end_width+bar_width)/2-chassis_to_hinge,0]) rotate([  0,0,270-side_angle]) translate([0,-secondaryArmLength/2 - pinRadius_v,0]) cube([bar_width,secondaryArmLength - pinRadius_v,bar_height], center=true);
        translate([ wheelHingeXDist, (end_width+bar_width)/2+chassis_to_hinge,0]) rotate([  0,0,270+side_angle]) translate([0,-secondaryArmLength/2 - pinRadius_v,0]) cube([bar_width,secondaryArmLength - pinRadius_v,bar_height], center=true);
    }
    
    /////////////////////////
    // Exclusion Zone Display
    turnRad = (end_width+bar_width)/2+chassis_to_hinge + wheelHingeXDist/tan(side_angle);
    turnRad_min = (end_width+bar_width)/2+chassis_to_hinge + wheelHingeXDist/tan(90-front_angle);
    sStrafeRad = wheelHingeXDist + ((end_width+bar_width)/2+chassis_to_hinge)/tan(front_angle);
    sStrafeRad_min = wheelHingeXDist + ((end_width+bar_width)/2+chassis_to_hinge)/tan(90-side_angle);
    
    // Map of the area the turning center cannot be in for one wheel
    module exclusionZone(location, flipX, flipY) {
        color(exclusionColor) translate(location) mirror([(flipX)?1:0,0,0]) mirror([0,(flipY)?1:0,0]) union() {
            // Arc under the wheel itself
            difference(){
                cylinder(1,hinge_to_wheel+tire_width,hinge_to_wheel+tire_width,center=true);
                cylinder(2,hinge_to_wheel,hinge_to_wheel,center=true);
                rotate([0,0,maxOutwardAngle])translate([-2000,-1000,-5])cube([2000,2000,10]);
            }
            // Wedges of exclusion- only calculate if they actually exist
            if( (side_angle+front_angle) < 90 ){
                intersection(){
                    rotate([0,0,maxOutwardAngle])translate([-2000,-1000,-0.5])cube([2000,2000,1]);
                    rotate([0,0,90-maxInwardAngle])translate([0,-1000,-0.5])cube([2000,2000,1]);
                }
                intersection(){
                    rotate([0,0,maxOutwardAngle])translate([0,-1000,-0.5])cube([2000,2000,1]);
                    rotate([0,0,90-maxInwardAngle])translate([-2000,-1000,-0.5])cube([2000,2000,1]);
                }
            }
        }
    }
    
    // Optionally display the exclusion zones for all wheels
    translate([0,0,-tire_radius-wheel_mount_offset])if (showTurnPointExclusionZones) {
        
        // Bars for exclusion zones along the normal turn and circle-strafe lines
        sStrafeBarOffset=min(500+sStrafeRad_min,(sStrafeRad+sStrafeRad_min)/2);
        sStrafeBarSize=min(1000,sStrafeRad-sStrafeRad_min);
        // Circle-strafe exclusion zones
        color(exclusionColor) translate([ sStrafeBarOffset,0,0])rotate([0,90,0])cylinder(sStrafeBarSize,2,2, center=true);
        color(exclusionColor) translate([ sStrafeRad_min,  0,0])sphere(2, center=true);
        color(exclusionColor) translate([-sStrafeBarOffset,0,0])rotate([0,90,0])cylinder(sStrafeBarSize,2,2, center=true);
        color(exclusionColor) translate([-sStrafeRad_min,  0,0])sphere(2, center=true);
        if(sStrafeRad > 1000){ // Circle-strafe exclusion zone is probably infinite- display as arrow, not bar
            color(exclusionColor) translate([ sStrafeRad_min+1000,0,0])rotate([0,90,0])cylinder(100,4,0, center=true);
            color(exclusionColor) translate([-sStrafeRad_min-1000,0,0])rotate([0,90,0])cylinder(100,0,4, center=true);
        } else {
            color(exclusionColor) translate([-sStrafeRad    ,0,0])sphere(2, center=true);
            color(exclusionColor) translate([ sStrafeRad    ,0,0])sphere(2, center=true);
        }
        
        // Exclusion zones for side-to-side turning
        wheelHingeDistance = (end_width+bar_width)/2+chassis_to_hinge;
        turnBarLength=(turnRad-turnRad_min);
        turnBarOffset=turnBarLength/2+turnRad_min;
        color(exclusionColor) translate([0,turnRad,0])sphere(2, center=true);
        color(exclusionColor) translate([0,turnBarOffset,0])rotate([90,0,0])cylinder(turnBarLength,2,2, center=true);
        color(exclusionColor) translate([0,turnRad_min,0])sphere(2, center=true);
            
        color(exclusionColor) translate([0,-turnRad_min,0])sphere(2, center=true);
        color(exclusionColor) translate([0,-turnBarOffset,0])rotate([90,0,0])cylinder(turnBarLength,2,2, center=true);
        color(exclusionColor) translate([0,-turnRad,0])sphere(2, center=true);
            
        // The full exclusion zone is a pair of wedges going out from each wheel.
        if(showFullExclusionZones ){
            exclusionZone([ wheelHingeXDist, wheelHingeYDist, 0], false, false);
            exclusionZone([ wheelHingeXDist,-wheelHingeYDist, 0], false,  true);
            exclusionZone([-wheelHingeXDist, wheelHingeYDist, 0], true,  false);
            exclusionZone([-wheelHingeXDist,-wheelHingeYDist, 0], true, true);
        }
    }
    
    ////////////////////
    // Dimensions Output
    
    echo(normal_turn_min_radius         =( ( turnRad )                                     /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    //echo(normal_turn_max_radius_past_frame=(turnRad_min                                    /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(strafe_turn_min_radius         =( ( sStrafeRad )                                  /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(strafe_turn_max_radius_past_frame=((sStrafeRad_min-wheelHingeXDist-wheelSupportWidth/2) /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    
    echo("Dimensions are to outsides of structural bars.");
    echo(structural_bar_width          =(bar_width                                         /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(wheel_support_width           =(wheelSupportWidth                                 /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(width_outside_edges_of_wheels =(max_vehicle_width                                 /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(length_outside_edges_of_wheels=((wheelHingeXDist*2+tire_radius*2)                 /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(total_length_pivot_to_pivot   =((wheelHingeXDist*2)                               /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(center_section_length         =((center_length+bar_width)                         /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(center_section_width          =(center_width                                      /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(front_section_length          =((endSupportLength-bar_width/2+wheelSupportWidth/2)/X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(front_section_width           =((endSupportSideOffset*2+bar_width)                /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    
    echo(wheel_hinge_to_wheel_center   =((hinge_to_wheel+tire_width/2)                     /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(wheel_hinge_x_position        =(wheelHingeXDist                                         /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
    echo(wheel_hinge_y_position        =(((end_width+bar_width)/2+chassis_to_hinge)        /X*dimensionDisplayMultiplier),dimensionDisplayUnits);
}



/////////////////
// ACCESSORIES //
/////////////////

seat_thickness = 2 * X;
seat_height = 16*X-seat_thickness - bar_height/2;
seat_width = (21.2+2)*X;
seat_length = 23.8*X;
seat_back_length = 18*X;
seat_back_angle = 9.3;

cargo_rack_bottom_offset = tire_radius - wheel_mount_offset + tire_buffer - bar_height/2;
cargo_rack_thickness = 1*X;

module seat(){
    // Cushion
    translate([0,0,seat_height])cube([seat_length,seat_width,seat_thickness], center=true);
    // Back
    translate([-seat_length/2,0,seat_height]) rotate([0,-90-seat_back_angle,0])translate([seat_back_length/2,0,0]) cube([seat_back_length,seat_width,seat_thickness], center=true);
    
    translate([ seat_length/2, center_width/4, seat_height/2+bar_height/2]) cube([bar_width,bar_width, seat_height-bar_height/2], center=true);
    translate([-seat_length/2, center_width/4, seat_height/2+bar_height/2]) cube([bar_width,bar_width, seat_height-bar_height/2], center=true);
    translate([ seat_length/2,-center_width/4, seat_height/2+bar_height/2]) cube([bar_width,bar_width, seat_height-bar_height/2], center=true);
    translate([-seat_length/2,-center_width/4, seat_height/2+bar_height/2]) cube([bar_width,bar_width, seat_height-bar_height/2], center=true);
    
    
    translate([0, center_width/4, seat_height]) cube([seat_length, bar_width, bar_width], center=true);
    translate([0,-center_width/4, seat_height]) cube([seat_length, bar_width, bar_width], center=true);
    
    translate([ seat_length/2, 0, seat_height]) cube([bar_width, center_width/2, bar_width], center=true);
    translate([-seat_length/2, 0, seat_height]) cube([bar_width, center_width/2, bar_width], center=true);
    
    module frameGrip() {
        c_thickness = 1; // Thickness of the plates holding the wheel arms to the main body
printingBuffer = 1.4; // Extra clearance needed for the wheel arm "C"s to clear the wheel supports on the work printer

        translate([0, (bar_width+printingBuffer+c_thickness)/2, printingBuffer/2]) cube([bar_width, c_thickness, bar_height], center=true);
        translate([0,-(bar_width+printingBuffer+c_thickness)/2, printingBuffer/2]) cube([bar_width, c_thickness, bar_height], center=true);
        
        
        x=bar_width+printingBuffer+c_thickness*2;
        translate([0, 0, bar_width/2+bar_height/2+printingBuffer/2])
            linear_extrude(height = bar_width, center = true, scale=[1,bar_width/x]) square([bar_width,x], center=true);
            //cube([bar_width,x,bar_width], center=true);
    }
    translate([ seat_length/2, center_width/4, 0]) frameGrip();
    translate([-seat_length/2, center_width/4, 0]) frameGrip();
    translate([ seat_length/2,-center_width/4, 0]) frameGrip();
    translate([-seat_length/2,-center_width/4, 0]) frameGrip();
    
    
}

module cargo_rack(){
    
    translate([-front_length/2, end_width/2,(cargo_rack_bottom_offset+bar_height)/2])cube([bar_height,bar_width,cargo_rack_bottom_offset], center=true);
    translate([-front_length/2,-end_width/2,(cargo_rack_bottom_offset+bar_height)/2])cube([bar_height,bar_width,cargo_rack_bottom_offset], center=true);
    translate([ front_length/2, end_width/2,(cargo_rack_bottom_offset+bar_height)/2])cube([bar_height,bar_width,cargo_rack_bottom_offset], center=true);
    translate([ front_length/2,-end_width/2,(cargo_rack_bottom_offset+bar_height)/2])cube([bar_height,bar_width,cargo_rack_bottom_offset], center=true);
    
    translate([0,0,cargo_rack_bottom_offset+bar_height/2]) cube([front_length+bar_height, end_width*2,cargo_rack_thickness], center=true);
    translate([-tire_width-front_length*.27,0,cargo_rack_bottom_offset+bar_height/2]) cube([front_length*0.6, center_width,cargo_rack_thickness], center=true);
}



///////////////////
// FULL ASSEMBLY //
///////////////////

rotate([0,0,360*$t]*((animateTurnPoint)?1:0))translate([-turnCenterX,-turnCenterY,0]*((animateTurnPoint)?1:0)) // For showing the vehicle spinning around the center of rotation
union(){
    wheel_demo(turnCenterX, turnCenterY);

    angle_frame(maxOutwardAngle,maxInwardAngle);
    //angle_frame(50,20);
    //angle_frame(55,35);

    //angle_frame(90-wheelRotation,wheelRotation);

    // Cargo rack
    //translate([-(center_length+front_length+bar_width)/2,0,0])cargo_rack();

    // Seats
    //!rotate([0,57.72+90,0])
 //   union(){
 //       translate([-seat_length/3, center_width/4,0]) seat();
 //       translate([-seat_length/3,-center_width/4,0]) seat();
 //     }
}
//wheel();
//translate([0,chassis_to_hinge,0]) rotate([0,90,0]) wheel_arm();
//wheel_support();


// Imagemagick command to create a gif from a set of dumped images:
// for x in frame*png; do echo -n "-page +0+0 $x "; done | cat <(echo -n "convert -delay 10 ") - <(echo " -loop 0 -resize 12.5% output2.gif") |sh





