"use strict"
$(document).ready(function() {
  Number.prototype.integer = function () {
    return Math[this < 0 ? 'ceil' : 'floor'](this);
  }

  var ros = null;
  var dontRestart = false;

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

  var metric = {
    joyStick: {
      subscribe: {
        topic: "/tycho/gui_page",
        type: "std_msgs/Int16",
        listener: null
      }
    },
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
        },
        temp: {
          gaugeType: "linear",
          name: "steering_temp",
          title: "Steering",
          units: "Temp&deg C",
          maxValue: 100,
          minValue: 0,
          rangeHighWarning: 60,
          rangeHighError: 80,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100"],
          FL: {
            canvasID: "wssTempFL",
            gaugeID: null
          },
          FR: {
            canvasID: "wssTempFR",
            gaugeID: null
          },
          BL: {
            canvasID: "wssTempBL",
            gaugeID: null
          },
          BR: {
            canvasID: "wssTempBR",
            gaugeID: null
          }
        }
      },
      drive: {
        amps: {
          gaugeType: "linear",
          name: "drive_amps",
          title: "Drive",
          units: "Amps",
          maxValue: 160,
          minValue: 0,
          rangeHighWarning: 100,
          rangeHighError: 130,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "20", "40", "60", "80", "100", "120", "140", "160"],
          FL: {
            canvasID: "wsdAmpsFL",
            gaugeID: null
          },
          FR: {
            canvasID: "wsdAmpsFR",
            gaugeID: null
          },
          BL: {
            canvasID: "wsdAmpsBL",
            gaugeID: null
          },
          BR: {
            canvasID: "wsdAmpsBR",
            gaugeID: null
          }
        },
        temp: {
          gaugeType: "linear",
          name: "drive_temp",
          title: "Drive",
          units: "Temp&deg C",
          maxValue: 100,
          minValue: 0,
          rangeHighWarning: 60,
          rangeHighError: 80,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100"],
          FL: {
            canvasID: "wsdTempFL",
            gaugeID: null
          },
          FR: {
            canvasID: "wsdTempFR",
            gaugeID: null
          },
          BL: {
            canvasID: "wsdTempBL",
            gaugeID: null
          },
          BR: {
            canvasID: "wsdTempBR",
            gaugeID: null
          }
        },
        rpm: {
          gaugeType: "linear",
          name: "drive_rpm",
          title: "Drive",
          units: "RPM",
          maxValue: 3000,
          minValue: 0,
          rangeHighWarning: 2000,
          rangeHighError: 2500,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "500", "1000", "1500", "2000", "2500", "3000"],
          FL: {
            canvasID: "wsdRpmFL",
            gaugeID: null
          },
          FR: {
            canvasID: "wsdRpmFR",
            gaugeID: null
          },
          BL: {
            canvasID: "wsdRpmBL",
            gaugeID: null
          },
          BR: {
            canvasID: "wsdRpmBR",
            gaugeID: null
          }
        }
      }
    },
    powerStatus: {
      subscribe: {
        topic: "/tycho/power_monitor",
        type: "tycho/PowerMonitor",
        listener: null
      },
      volts: {
        battery_1: {
          gaugeType: "linear",
          name: "battery_1_v",
          title: "Battery 1",
          units: "Volts",
          maxValue: 30,
          minValue: 0,
          rangeHighWarning: 15,
          rangeHighError: 18,
          rangeLowWarning: 10,
          rangeLowError: 8,
          majorTicks: ["0", "5", "10", "15", "20", "25", "30"],
          canvasID: "psvBat1",
          gaugeID: null
        },
        battery_2: {
          gaugeType: "linear",
          name: "battery_2_v",
          title: "Battery 2",
          units: "Volts",
          maxValue: 30,
          minValue: 0,
          rangeHighWarning: 15,
          rangeHighError: 18,
          rangeLowWarning: 10,
          rangeLowError: 8,
          majorTicks: ["0", "5", "10", "15", "20", "25", "30"],
          canvasID: "psvBat2",
          gaugeID: null
        },
        battery_3: {
          gaugeType: "linear",
          name: "battery_3_v",
          title: "Battery 3",
          units: "Volts",
          maxValue: 30,
          minValue: 0,
          rangeHighWarning: 15,
          rangeHighError: 18,
          rangeLowWarning: 10,
          rangeLowError: 8,
          majorTicks: ["0", "5", "10", "15", "20", "25", "30"],
          canvasID: "psvBat3",
          gaugeID: null
        },
        battery_4: {
          gaugeType: "linear",
          name: "battery_4_v",
          title: "Battery 4",
          units: "Volts",
          maxValue: 30,
          minValue: 0,
          rangeHighWarning: 15,
          rangeHighError: 18,
          rangeLowWarning: 10,
          rangeLowError: 8,
          majorTicks: ["0", "5", "10", "15", "20", "25", "30"],
          canvasID: "psvBat4",
          gaugeID: null
        },
        panel_12v: {
          gaugeType: "linear",
          name: "panel_12v_v",
          title: "12V Panel",
          units: "Volts",
          maxValue: 20,
          minValue: 0,
          rangeHighWarning: 15,
          rangeHighError: 18,
          rangeLowWarning: 11,
          rangeLowError: 8,
          majorTicks: ["0", "5", "10", "15", "20"],
          canvasID: "psvPanel12v",
          gaugeID: null
        },
        panel_5v: {
          gaugeType: "linear",
          name: "panel_5v_v",
          title: "5V Panel",
          units: "Volts",
          maxValue: 12,
          minValue: 0,
          rangeHighWarning: 6,
          rangeHighError: 8,
          rangeLowWarning: 4,
          rangeLowError: 3,
          majorTicks: ["0", "2", "4", "6", "8", "10", "12"],
          canvasID: "psvPanel5v",
          gaugeID: null
        }
      },
      amps: {
        master: {
          gaugeType: "linear",
          name: "master_a",
          title: "Master",
          units: "Amps",
          maxValue: 320,
          minValue: 0,
          rangeHighWarning: 200,
          rangeHighError: 240,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "40", "80", "120", "160", "200", "240", "280", "320"],
          canvasID: "psaMaster",
          gaugeID: null
        },
        front_relay: {
          gaugeType: "linear",
          name: "front_relay_a",
          title: "Front Relay",
          units: "Amps",
          maxValue: 320,
          minValue: 0,
          rangeHighWarning: 200,
          rangeHighError: 240,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "40", "80", "120", "160", "200", "240", "280", "320"],
          canvasID: "psaFrontRelay",
          gaugeID: null
        },
        rear_relay: {
          gaugeType: "linear",
          name: "rear_relay_a",
          title: "Rear Relay",
          units: "Amps",
          maxValue: 320,
          minValue: 0,
          rangeHighWarning: 200,
          rangeHighError: 240,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "40", "80", "120", "160", "200", "240", "280", "320"],
          canvasID: "psaRearRelay",
          gaugeID: null
        },
        panel_display: {
          gaugeType: "linear",
          name: "panel_display_a",
          title: "Panel Display",
          units: "Amps",
          maxValue: 10,
          minValue: 0,
          rangeHighWarning: 5,
          rangeHighError: 7,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "2", "4", "6", "8", "10"],
          canvasID: "psaPanelDisplay",
          gaugeID: null
        },
        panel_12v: {
          gaugeType: "linear",
          name: "panel_12v_a",
          title: "12V Panel",
          units: "Amps",
          maxValue: 10,
          minValue: 0,
          rangeHighWarning: 5,
          rangeHighError: 7,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "2", "4", "6", "8", "10"],
          canvasID: "psaPanel12v",
          gaugeID: null
        },
        panel_5v: {
          gaugeType: "linear",
          name: "panel_5v_a",
          title: "5V Panel",
          units: "Amps",
          maxValue: 10,
          minValue: 0,
          rangeHighWarning: 5,
          rangeHighError: 7,
          rangeLowWarning: null,
          rangeLowError: null,
          majorTicks: ["0", "2", "4", "6", "8", "10"],
          canvasID: "psaPanel5v",
          gaugeID: null
        }
      }
    }
  }
  
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
    var gID = new LinearGauge(Object.assign({}, linearGaugeParams, {
      title: gD.title,
      units: gD.units,
      renderTo: canvasID,
      minValue: gD.minValue,
      maxValue: gD.maxValue,
      majorTicks: gD.majorTicks,
      highlights: highLights,
      value: gD.minValue
    })).draw();
    return(gID);
  }

  function initGauge(grpData, gaugeID) {
    switch(grpData.gaugeType) {
      case 'linear':
        if (gaugeID == null) {
          grpData.gaugeID = initLinearGauge(grpData,  grpData.canvasID);
        }
        else {
          for (var i = 0;  i < gaugeID.length; i++) {
            grpData[gaugeID[i]].gaugeID = initLinearGauge(grpData,  grpData[gaugeID[i]].canvasID);
          }
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

    metric.wheelStatus.steering.amps[wheelConversion[msg.wheel_id.integer()]]
      .gaugeID.value = msg.steering_amps;
    metric.wheelStatus.steering.temp[wheelConversion[msg.wheel_id.integer()]]
      .gaugeID.value = msg.steering_temp;
    metric.wheelStatus.drive.amps[wheelConversion[msg.wheel_id.integer()]]
      .gaugeID.value = msg.drive_amps;
    metric.wheelStatus.drive.temp[wheelConversion[msg.wheel_id.integer()]]
      .gaugeID.value = msg.drive_temp;
    metric.wheelStatus.drive.rpm[wheelConversion[msg.wheel_id.integer()]]
      .gaugeID.value = msg.drive_rpm;
  }

  function updatePower(msg) {
    metric.powerStatus.volts.battery_1.gaugeID.value = msg[metric.powerStatus.volts.battery_1.name];
    metric.powerStatus.volts.battery_2.gaugeID.value = msg[metric.powerStatus.volts.battery_2.name];
    metric.powerStatus.volts.battery_3.gaugeID.value = msg[metric.powerStatus.volts.battery_3.name];
    metric.powerStatus.volts.battery_4.gaugeID.value = msg[metric.powerStatus.volts.battery_4.name];
    metric.powerStatus.volts.panel_12v.gaugeID.value = msg[metric.powerStatus.volts.panel_12v.name];
    metric.powerStatus.volts.panel_5v.gaugeID.value = msg[metric.powerStatus.volts.panel_5v.name];
    metric.powerStatus.amps.master.gaugeID.value = msg[metric.powerStatus.amps.master.name];
    metric.powerStatus.amps.front_relay.gaugeID.value = msg[metric.powerStatus.amps.front_relay.name];
    metric.powerStatus.amps.rear_relay.gaugeID.value = msg[metric.powerStatus.amps.rear_relay.name];
    metric.powerStatus.amps.panel_display.gaugeID.value = msg[metric.powerStatus.amps.panel_display.name];
    metric.powerStatus.amps.panel_12v.gaugeID.value = msg[metric.powerStatus.amps.panel_12v.name];
    metric.powerStatus.amps.panel_5v.gaugeID.value = msg[metric.powerStatus.amps.panel_5v.name];
  }

  function updateJoyStick(msg) {
    console.info('buttons.data: ', msg.data);
    if (msg.data === 0) {
      $('.content').css({"display": "none"});
      $('#rosWheelStatus').css({"display": "block"});
    }
    if (msg.data === 1) {
      $('.content').css({"display": "none"});
      $('#rosPowerStatus').css({"display": "block"});
    }
  }

  function wheelStatusPage() {
    var ws = metric.wheelStatus;

    $('.content').css({"display": "none"});
    $('#rosWheelStatus').css({"display": "block"});

    initGauge(ws.steering.amps, ['FL', 'FR', 'BL', 'BR'])
    initGauge(ws.steering.temp, ['FL', 'FR', 'BL', 'BR'])
    initGauge(ws.drive.amps, ['FL', 'FR', 'BL', 'BR'])
    initGauge(ws.drive.temp, ['FL', 'FR', 'BL', 'BR'])
    initGauge(ws.drive.rpm, ['FL', 'FR', 'BL', 'BR'])

    ws.subscribe.listener = subscribeToTopic(ros, ws.subscribe.topic, ws.subscribe.type);

    ws.subscribe.listener.subscribe(updateWheel);
  }

  function powerStatusPage() {
    var ws = metric.powerStatus;

//    $('.content').css({"display": "none"});
//    $('#rosPowerStatus').css({"display": "block"});

    initGauge(ws.volts.battery_1, null)
    initGauge(ws.volts.battery_2, null)
    initGauge(ws.volts.battery_3, null)
    initGauge(ws.volts.battery_4, null)
    initGauge(ws.volts.panel_12v, null)
    initGauge(ws.volts.panel_5v, null)
    initGauge(ws.amps.master, null)
    initGauge(ws.amps.front_relay, null)
    initGauge(ws.amps.rear_relay, null)
    initGauge(ws.amps.panel_display, null)
    initGauge(ws.amps.panel_12v, null)
    initGauge(ws.amps.panel_5v, null)

    ws.subscribe.listener = subscribeToTopic(ros, ws.subscribe.topic, ws.subscribe.type);

    ws.subscribe.listener.subscribe(updatePower);
  }

  function joyStickStatus() {
    var ws = metric.joyStick;

    ws.subscribe.listener = subscribeToTopic(ros, ws.subscribe.topic, ws.subscribe.type);

    ws.subscribe.listener.subscribe(updateJoyStick);
  }

//Event handler for when connection to Tycho succeeds.
  function rosConnectionSuccess() {
    console.log(new Date().getTime() + ': Connected to websocket server.');
    wheelStatusPage();
    powerStatusPage();
    joyStickStatus();
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

  function initRosConnection() {
    ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    $('#rosConnecting p').append('.');
    ros.on('connection', rosConnectionSuccess);
    ros.on('error', rosConnectionError);
    ros.on('close', rosConnectionClosed);
  }

  $('.content').css({"display": "none"});
  $('#rosConnect').css({"display": "block"});


  initRosConnection();

  $("#btnStop").click(function() {
    dontRestart = true;
    metric.wheelStatus.subscribe.listener.unsubscribe();
    ros.close();
  });

  $("#btnJoy1").click(function() {
    updateJoyStick({data: 0});
  });
  $("#btnJoy2").click(function() {
    updateJoyStick({data: 1});
  });
});
