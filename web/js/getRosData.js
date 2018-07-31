"use strict"
  var metric = {
    wheelStatus: {
      subscribe: {
        topic: "/tycho/wheel_status",
        type: "tycho/WheelStatus",
        listener: null
      },
      steering: {
        amps: {
          gaugeType: "linear",
          name: "steering_amps",
          title: "Steering",
          units: "Amps",
          maxValue: 30,
          minValue: 0,
          rangeHighWarning: 18,
          rangeHighError: 22,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "5", "10", "15", "20", "25", "30"],
          FL: {
            canvasID: "wssAmpsFL",
            gaugeID: null
          },
          FR: {
            canvasID: "wssAmpsFR",
            gaugeID: null
          },
          BL: {
            canvasID: "wssAmpsBL",
            gaugeID: null
          },
          BR: {
            canvasID: "wssAmpsBR",
            gaugeID: null
          }
        }
      }
    }
  }
  
$(document).ready(function() {
  Number.prototype.integer = function () {
    return Math[this < 0 ? 'ceil' : 'floor'](this);
  }

  var ros = null;

  var gaugeColorWarning = '#FF0';
  var gaugeColorError = '#F00';

  var linearGaugeParams = {
    barBeginCircle: false,
    barStrokeWidth: 0,
    borderInnerWidth: 1,
    borderMiddleWidth: 3,
    borderOuterWidth: 1,
    borderShadowWidth: 0,
    borders: true,
    colorBar: "#444",
    colorBarProgress: "#BBB",
    colorBorderInner: "#FFF",
    colorBorderMiddle: "#777",
    colorBorderOuter: "#FFF",
    colorNumbers: "#FFF",
    colorPlate: "#000",
    colorStrokeTicks: "#FFF",
    colorTitle: "#FFF",
    colorUnits: "#FFF",
    colorValueBoxBackground: "#FFF",
    colorValueText: "#111",
    fontNumbersSize: 40,
    fontTitleSize: "50",
    fontUnitsSize: "40",
    fontValueSize: 35,
    height: 275,
    highlightsWidth: 15,
    minorTicks: 4,
    needleSide: "left",
    needleType: "arrow",
    needleWidth: 5,
    numberSide: "left",
    strokeTicks: true,
    tickSide: "left",
    width: 68
  }

  var html_page = [
    '<div id="rosConnecting">' +
      '<p>Connecting to Tycho' +
    '</div>',
    '<table class="rosTable"><tr>' +    // Wheel Status Page
      '<td class="rosTD" style="width:344px;">' +
        '<table class="rosTable wsTable"><tr class="rosTR wsTR"><td class="rosTD">' +
          '<canvas id="wssAmpsFL" class="fLeft"></canvas>' +
        '</td></tr><tr class="rosTR wsTR"><td class="rosTD">' + 
          '<canvas id="wssAmpsBL" class="fLeft"></canvas>' +
        '</td></tr></table>' + 
      '</td>' +
      '<td class="rosTD" style="width:336px;">' +
        '<div id="roverWheelStatus">\n' +
          '<img src="images/tychoChassis.png"></img>\n' +
          '<img id="wheelFL" class="wss" src="images/tychoWheelFL.png"></img>\n' +
          '<img id="wheelFR" class="wss" src="images/tychoWheelFR.png"></img>\n' +
          '<img id="wheelBL" class="wss" src="images/tychoWheelBL.png"></img>\n' +
          '<img id="wheelBR" class="wss" src="images/tychoWheelBR.png"></img>\n' +
          '<div id="wssAngleFL" class="wssAngle"></div>\n' +
          '<div id="wssAngleFR" class="wssAngle"></div>\n' +
          '<div id="wssAngleBL" class="wssAngle"></div>\n' +
          '<div id="wssAngleBR" class="wssAngle"></div>\n' +
        '</div>\n' +
      '</td>' +
      '<td class="rosTD" style="width:344px;">' +
        '<table class="rosTable wsTable"><tr class="rosTR wsTR"><td class="rosTD">' +
          '<canvas id="wssAmpsFR" class="fRight"></canvas>' +
        '</td></tr><tr class="rosTR wsTR"><td class="rosTD">' + 
          '<canvas id="wssAmpsBR" class="fRight"></canvas>' +
        '</td></tr></table>' + 
    '</tr></table>'
  ];

  function subscribeToTopic(ros, topic, type) {
    var listener = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: type
    });
    return(listener);
  }

  function initLinearGauge(gD, canvasID) {
    var highLights = [];
    if (gD.rangeLowWarning !== null) {
      highLights.push({
        "from": gD.rangeLowError !== null ? gD.rangeLowError : gD.minValue,
        "to": gD.rangeLowWarning,
        "color": gaugeColorWarning
      });
    }
    if (gD.rangeLowError !== null) {
      highLights.push({
        "from": gD.minValue,
        "to": gD.rangeLowError,
        "color": gaugeColorError
     });
    }
    if (gD.rangeHighWarning !== null) {
      highLights.push({
        "from": gD.rangeHighWarning,
        "to": gD.rangeHighError !== null ? gD.rangeHighError : gD.maxValue,
        "color": gaugeColorWarning
      });
    }
    if (gD.rangeHighError !== null) {
      highLights.push({
        "from": gD.rangeHighError,
        "to": gD.maxValue,
        "color": gaugeColorError
      });
    }
/*    console.info("gauge object: ", Object.assign({}, linearGaugeParams, {
    title: gD.title,
    units: gD.units,
    renderTo: canvasID,
    minValue: gD.minValue,
    maxValue: gD.maxValue,
    majorTicks: gD.majorTicks,
    highlights: highLights,
    value: gD.minValue
    }));
*/    return(new LinearGauge(Object.assign({}, linearGaugeParams, {
    title: gD.title,
    units: gD.units,
    renderTo: canvasID,
    minValue: gD.minValue,
    maxValue: gD.maxValue,
    majorTicks: gD.majorTicks,
    highlights: highLights,
    value: gD.minValue
    })).draw())
  }

  function initGauge(grpData, gaugeID) {
    switch(grpData.gaugeType) {
      case 'linear':
//console.info(' grpData: ', grpData, '\ngaugeID: ', gaugeID);
        for (var i = 0;  i < gaugeID.length; i++) {
          grpData[gaugeID[i]].gaugeID = initLinearGauge(grpData,  grpData[gaugeID[i]].canvasID);
        }
        break;
      default:
        console.log('Gauge type ("' + grpData.gaugeType + '") not recognized!\n' +
          'Group Data: ', grpData, '\nGauge ID: ', gaugeID);
    } 
  } 

  function updateWheel(msg) {
    if (typeof msg.wheel_id !== 'number' ||
          msg.wheel_id.integer() < 1 ||
          msg.wheel_id.integer() > 4) {
       console.error("Message 'wheel_id' ('msg.wheel_id') not valid " +
         "or not defined.\n Message: ", msg);
    } 

    var wheelConversion = ['', 'FL', 'FR', 'BL', 'BR'];

    $("#wheel" + wheelConversion[msg.wheel_id])
      .css({'-moz-transform': 'rotate(' + msg.steering_angle.integer() + 'deg)'})
    $("#wssAngle" + wheelConversion[msg.wheel_id])
      .html(msg.steering_angle.integer() + '&deg;');

//console.info("wheel_id: ", msg, "wheel: '" + wheelConversion[msg.wheel_id.integer()] + "'\nobject:\n", metric.wheelStatus.steering.amps);
    metric.wheelStatus.steering.amps[wheelConversion[msg.wheel_id.integer()]].gaugeID.value = msg.steering_amps;
  }

  function wheelStatusPage() {
    var ws = metric.wheelStatus;

    $('#content').html(html_page[1]);

    ws.subscribe.listener = subscribeToTopic(ros, ws.subscribe.topic, ws.subscribe.type);

    var wssa = ws.steering.amps;
    wssa.FL.gaugeID = initGauge(wssa, ['FL', 'FR', 'BL', 'BR'])

    ws.subscribe.listener.subscribe(updateWheel);
  }

//Event handler for when connection to Tycho succeeds.
  function rosConnectionSuccess() {
    console.log(new Date().getTime() + ': Connected to websocket server.');
    wheelStatusPage();
  }

//Event handler for when connection to Tycho encounted an error.
  function rosConnectionError(error) {
    console.log(new Date().getTime() + ': Error connecting to websocket server: ', error);
  }

//Event handler for when connection to Tycho is closed.
  function rosConnectionClosed(details) {
    console.log(new Date().getTime() + ': Connection to websocket server closed. Details: ', details);
    if (!dontRestart) {
      setTimeout(function() { // Wait a second before trying to reconnect
        console.log(new Date().getTime() + 'Attempting reconnect websocket server');
        initRosConnection();
      }, 1000);
    }
  }
  var dontRestart = false;
  function initRosConnection() {
    ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    $('#rosConnecting p').append('.');
    ros.on('connection', rosConnectionSuccess);
    ros.on('error', rosConnectionError);
    ros.on('close', rosConnectionClosed);
  }

  $('#content').html(html_page[0]);
  initRosConnection();

  $("#btnStop").click(function() {
    dontRestart = true;
    metric.wheelStatus.subscribe.listener.unsubscribe();
    ros.close();
  });

});
