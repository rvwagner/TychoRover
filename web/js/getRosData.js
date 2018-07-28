"use strict"
$(document).ready(function() {
  Number.prototype.integer = function () {
    return Math[this < 0 ? 'ceil' : 'floor'](this);
  }

var metric = {
  "wheelStatus": {
    "subscribe": {
      "topic": "/tycho/wheel_status",
      "type": "tycho/WheelStatus"
    },
    "listener": null,
    "steering": {
      "amps": {
        "gaugeType": "linear",
        "name": "steering_amps",
        "title": "Steering",
        "units": "Amps",
        "maxValue": 30,
        "minValue": 0,
        "rangeHighWarning": 18,
        "rangeHighError": 22,
        "rangeLowWarning": null,
        "rangeLowError": null,
        "majorTicks": [ "0", "5", "10", "15", "20", "25", "30" ],
        "FL": {
          "divID": "wssAmpsFL",
          "gaugeID": null
        },
        "FR": {
          "divID": "wssAmpsFR",
          "gaugeID": null
        },
        "BL": {
          "divID": "wssAmpsBL",
          "gaugeID": null
        },
        "BR": {
          "divID": "wssAmpsBR",
          "gaugeID": null
        }
      }
    }
  },
}

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

  var html_page = [null,
    '<table class="rosTable"><tr>' +    // Wheel Status Page
      '<td class="rosTD" style="width:344px;">' +
        '<table class="rosTable wsTable"><tr class="rosTR wsTR"><td class="rosTD">' +
          '<canvas id="wssAmpsFL"></canvas>' +
        '</td></tr><tr class="rosTR wsTR"><td class="rosTD">' + 
          '<canvas id="wssAmpsBL"></canvas>' +
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
          '<canvas id="wssAmpsFR"></canvas>' +
        '</td></tr><tr class="rosTR wsTR"><td class="rosTD">' + 
          '<canvas id="wssAmpsBR"></canvas>' +
        '</td></tr></table>' + 
    '</tr></table>'
  ];

  function initRosConnection() {
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    ros.on('connection', function() {
      console.log('Connected to websocket server.');
    });

    ros.on('error', function(error) {
      console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function(details) {
      console.log('Connection to websocket server closed. Details: ', details);
    });
 
    return(ros);
  }

  function subscribeToTopic(ros, topic, type) {
    var listener = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: type
    });
    return(listener);
  }

  function updateWheel(msg, test) {
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

    if (msg.wheel_id.integer() === 1) wssAmpsFL.value = msg.drive_amps;
  }

  function initWssAmpGauge(canvasID) {
    return(new LinearGauge(Object.assign({}, linearGaugeParams, {
    title: "Steering",
    units: "Amps",
    renderTo: canvasID,
    minValue: 0,
    maxValue: 30,
    majorTicks: [ "0", "5", "10", "15", "20", "25", "30" ],
    highlights: [
      {
        "from": 18, "to": 22, "color": "rgba(255, 255, 0, .75)"
      },
      {
        "from": 22, "to": 30, "color": "rgba(255, 0, 0, .75)"
      }
    ],
    value: 0
    })).draw())
  }

var wssAmpsFL;

  function initWheelStatus() {
    wssAmpsFL = initWssAmpGauge('wssAmpsFL')
  }

  $('#content').html(html_page[1]);

  var rS= initRosConnection();

  var lWheelStatus = subscribeToTopic(rS, '/tycho/wheel_status', 'tycho/WheelStatus');

  initWheelStatus();

  lWheelStatus.subscribe(updateWheel);

  $("#btnStop").click(function() {
    lWheelStatus.unsubscribe();
    rS.close();
  });

});
