/**
 * Created by ryeubi on 2015-08-31.
 * Updated 2017.03.06
 * Made compatible with Thyme v1.7.2
 */
var net = require('net');
var moment = require('moment');
var fs = require('fs');
const {exec} = require('child_process');

var mavlink = require('./mavlibrary/mavlink.js');
var MAVLink20Processor = require('./mavlibrary/mavlink20.js');

var _server = null;

var socket_mav = null;
global.mavPort = null;

var mavPortNum = '/dev/ttyAMA0';
var mavBaudrate = '115200';

let mqtt = require('mqtt');


var aggr_content = {};

function send_aggr_to_Mobius(topic, content_each, gap) {
    if (aggr_content.hasOwnProperty(topic)) {
        var timestamp = moment().format('YYYY-MM-DDTHH:mm:ssSSS');
        aggr_content[topic][timestamp] = content_each;
    } else {
        aggr_content[topic] = {};
        timestamp = moment().format('YYYY-MM-DDTHH:mm:ssSSS');
        aggr_content[topic][timestamp] = content_each;

        setTimeout(function () {
            sh_adn.crtci(topic + '?rcn=0', 0, aggr_content[topic], null, function () {
            });

            delete aggr_content[topic];
        }, gap, topic);
    }
}

function mavlinkGenerateMessage(sysId, type, params) {
    const mavlinkParser = new MAVLink(null/*logger*/, sysId, 0);
    try {
        var mavMsg = null;
        var genMsg = null;

        var targetCompId = (params.targetCompId == undefined) ?
            0 :
            params.targetCompId;

        switch (type) {
            // MESSAGE ////////////////////////////////////
            case mavlink.MAVLINK_MSG_ID_PING:
                mavMsg = new mavlink.messages.ping(params.time_usec, params.seq, params.target_system, params.target_component);
                break;
            case mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                mavMsg = new mavlink.messages.heartbeat(params.type,
                    params.autopilot,
                    params.base_mode,
                    params.custom_mode,
                    params.system_status,
                    params.mavlink_version);
                break;
            case mavlink.MAVLINK_MSG_ID_GPS_RAW_INT:
                mavMsg = new mavlink.messages.gps_raw_int(params.time_usec,
                    params.fix_type,
                    params.lat,
                    params.lon,
                    params.alt,
                    params.eph,
                    params.epv,
                    params.vel,
                    params.cog,
                    params.satellites_visible,
                    params.alt_ellipsoid,
                    params.h_acc,
                    params.v_acc,
                    params.vel_acc,
                    params.hdg_acc);
                break;
            case mavlink.MAVLINK_MSG_ID_ATTITUDE:
                mavMsg = new mavlink.messages.attitude(params.time_boot_ms,
                    params.roll,
                    params.pitch,
                    params.yaw,
                    params.rollspeed,
                    params.pitchspeed,
                    params.yawspeed);
                break;
            case mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                mavMsg = new mavlink.messages.global_position_int(params.time_boot_ms,
                    params.lat,
                    params.lon,
                    params.alt,
                    params.relative_alt,
                    params.vx,
                    params.vy,
                    params.vz,
                    params.hdg);
                break;
            case mavlink.MAVLINK_MSG_ID_SYS_STATUS:
                mavMsg = new mavlink.messages.sys_status(params.onboard_control_sensors_present,
                    params.onboard_control_sensors_enabled,
                    params.onboard_control_sensors_health,
                    params.load,
                    params.voltage_battery,
                    params.current_battery,
                    params.battery_remaining,
                    params.drop_rate_comm,
                    params.errors_comm,
                    params.errors_count1,
                    params.errors_count2,
                    params.errors_count3,
                    params.errors_count4);
                break;
            case mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                mavMsg = new mavlink.messages.param_request_read(params.target_system,
                    params.target_component,
                    params.param_id,
                    params.param_index);
                break;
        }
    } catch (e) {
        console.log('MAVLINK EX:' + e);
    }

    if (mavMsg) {
        genMsg = Buffer.from(mavMsg.pack(mavlinkParser));
    }

    return genMsg;
}

function sendDroneMessage(type, params) {
    try {
        var msg = mavlinkGenerateMessage(my_system_id, type, params);
        if (msg == null) {
            console.log("mavlink message is null");
        } else {
            mavPortData(msg);
        }
    } catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}

var dji = {};
var params = {};

var SerialPort = require('serialport');

function mavPortOpening() {
    if (mavPort == null) {
        mavPort = new SerialPort(mavPortNum, {
            baudRate: parseInt(mavBaudrate, 10),
        });

        mavPort.on('open', mavPortOpen);
        mavPort.on('close', mavPortClose);
        mavPort.on('error', mavPortError);
        mavPort.on('data', mavPortData);
    } else {
        if (mavPort.isOpen) {

        } else {
            mavPort.open();
        }
    }
}

function mavPortOpen() {
    console.log('mavPort open. ' + mavPortNum + ' Data rate: ' + mavBaudrate);
}

function mavPortClose() {
    console.log('mavPort closed.');

    setTimeout(mavPortOpening, 2000);
}

function mavPortError(error) {
    var error_str = error.toString();
    console.log('[mavPort error]: ' + error.message);
    if (error_str.substring(0, 14) == "Error: Opening") {

    } else {
        console.log('mavPort error : ' + error);
    }

    setTimeout(mavPortOpening, 2000);
}


var mavStrFromDrone = '';
var mavStrFromDroneLength = 0;
var mavVersion = 'unknown';
var mavVersionCheckFlag = false;

function mavPortData(data) {
    mavStrFromDrone += data.toString('hex').toLowerCase();
    // console.log(mavStrFromDrone)

    while (mavStrFromDrone.length > 20) {
        if (!mavVersionCheckFlag) {
            var stx = mavStrFromDrone.substr(0, 2);
            if (stx === 'fe') {
                var len = parseInt(mavStrFromDrone.substr(2, 2), 16);
                var mavLength = (6 * 2) + (len * 2) + (2 * 2);
                var sysid = parseInt(mavStrFromDrone.substr(6, 2), 16);
                var msgid = parseInt(mavStrFromDrone.substr(10, 2), 16);

                if (msgid === 0 && len === 9) { // HEARTBEAT
                    mavVersionCheckFlag = true;
                    mavVersion = 'v1';
                }

                if ((mavStrFromDrone.length) >= mavLength) {
                    var mavPacket = mavStrFromDrone.substr(0, mavLength);

                    mavStrFromDrone = mavStrFromDrone.substr(mavLength);
                    mavStrFromDroneLength = 0;
                } else {
                    break;
                }
            } else if (stx === 'fd') {
                len = parseInt(mavStrFromDrone.substr(2, 2), 16);
                mavLength = (10 * 2) + (len * 2) + (2 * 2);

                sysid = parseInt(mavStrFromDrone.substr(10, 2), 16);
                msgid = parseInt(mavStrFromDrone.substr(18, 2) + mavStrFromDrone.substr(16, 2) + mavStrFromDrone.substr(14, 2), 16);

                if (msgid === 0 && len === 9) { // HEARTBEAT
                    mavVersionCheckFlag = true;
                    mavVersion = 'v2';
                }
                if (mavStrFromDrone.length >= mavLength) {
                    mavPacket = mavStrFromDrone.substr(0, mavLength);

                    mavStrFromDrone = mavStrFromDrone.substr(mavLength);
                    mavStrFromDroneLength = 0;
                } else {
                    break;
                }
            } else {
                mavStrFromDrone = mavStrFromDrone.substr(2);
            }
        } else {
            stx = mavStrFromDrone.substr(0, 2);
            if (mavVersion === 'v1' && stx === 'fe') {
                len = parseInt(mavStrFromDrone.substr(2, 2), 16);
                mavLength = (6 * 2) + (len * 2) + (2 * 2);

                if ((mavStrFromDrone.length) >= mavLength) {
                    mavPacket = mavStrFromDrone.substr(0, mavLength);

                    mqtt_client.publish(my_cnt_name, Buffer.from(mavPacket, 'hex'));
                    send_aggr_to_Mobius(my_cnt_name, mavPacket, 2000);
                    setTimeout(parseMavFromDrone, 0, mavPacket);

                    mavStrFromDrone = mavStrFromDrone.substr(mavLength);
                    mavStrFromDroneLength = 0;
                } else {
                    break;
                }
            } else if (mavVersion === 'v2' && stx === 'fd') {
                len = parseInt(mavStrFromDrone.substr(2, 2), 16);
                mavLength = (10 * 2) + (len * 2) + (2 * 2);

                if (mavStrFromDrone.length >= mavLength) {
                    mavPacket = mavStrFromDrone.substr(0, mavLength);

                    mqtt_client.publish(my_cnt_name, Buffer.from(mavPacket, 'hex'));
                    send_aggr_to_Mobius(my_cnt_name, mavPacket, 2000);
                    setTimeout(parseMavFromDrone, 0, mavPacket);

                    mavStrFromDrone = mavStrFromDrone.substr(mavLength);
                    mavStrFromDroneLength = 0;
                } else {
                    break;
                }
            } else {
                mavStrFromDrone = mavStrFromDrone.substr(2);
            }
        }
    }
}

var fc = {};
try {
    fc = JSON.parse(fs.readFileSync('fc_data_model.json', 'utf8'));
} catch (e) {
    fc.heartbeat = {};
    fc.heartbeat.type = 2;
    fc.heartbeat.autopilot = 3;
    fc.heartbeat.base_mode = 0;
    fc.heartbeat.custom_mode = 0;
    fc.heartbeat.system_status = 0;
    fc.heartbeat.mavlink_version = 1;

    fc.global_position_int = {};
    fc.global_position_int.time_boot_ms = 123456789;
    fc.global_position_int.lat = 0;
    fc.global_position_int.lon = 0;
    fc.global_position_int.alt = 0;
    fc.global_position_int.vx = 0;
    fc.global_position_int.vy = 0;
    fc.global_position_int.vz = 0;
    fc.global_position_int.hdg = 65535;

    fc.wp_yaw_behavior = {};
    fc.wp_yaw_behavior.value = 0;
    fc.wp_yaw_behavior.count = 0;
    fc.wp_yaw_behavior.index = 0;
    fc.wp_yaw_behavior.id = 0;
    fc.wp_yaw_behavior.type = 0;

    fs.writeFileSync('fc_data_model.json', JSON.stringify(fc, null, 4), 'utf8');
}

var flag_base_mode = 0;

function parseMavFromDrone(mavPacket) {
    try {
        var ver = mavPacket.substr(0, 2);
        if (ver == 'fd') {
            var sysid = mavPacket.substr(10, 2).toLowerCase();
            var msgid = mavPacket.substr(18, 2) + mavPacket.substr(16, 2) + mavPacket.substr(14, 2);
            var base_offset = 20;
        } else {
            sysid = mavPacket.substr(6, 2).toLowerCase();
            msgid = mavPacket.substr(10, 2).toLowerCase();
            base_offset = 12;
        }

        var sys_id = parseInt(sysid, 16);
        var msg_id = parseInt(msgid, 16);

        var cur_seq = parseInt(mavPacket.substr(4, 2), 16);

        if (msg_id == mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT) { // #33
            var time_boot_ms = mavPacket.substr(base_offset, 8).toLowerCase();
            base_offset += 8;
            var lat = mavPacket.substr(base_offset, 8).toLowerCase();
            base_offset += 8;
            var lon = mavPacket.substr(base_offset, 8).toLowerCase();
            base_offset += 8;
            var alt = mavPacket.substr(base_offset, 8).toLowerCase();
            base_offset += 8;
            var relative_alt = mavPacket.substr(base_offset, 8).toLowerCase();

            fc.global_position_int.time_boot_ms = Buffer.from(time_boot_ms, 'hex').readUInt32LE(0);
            fc.global_position_int.lat = Buffer.from(lat, 'hex').readInt32LE(0);
            fc.global_position_int.lon = Buffer.from(lon, 'hex').readInt32LE(0);
            fc.global_position_int.alt = Buffer.from(alt, 'hex').readInt32LE(0);
            fc.global_position_int.relative_alt = Buffer.from(relative_alt, 'hex').readInt32LE(0);

            muv_mqtt_client.publish(muv_pub_fc_gpi_topic, JSON.stringify(fc.global_position_int));
        } else if (msg_id == mavlink.MAVLINK_MSG_ID_HEARTBEAT) { // #00 : HEARTBEAT
            var custom_mode = mavPacket.substr(base_offset, 8).toLowerCase();
            base_offset += 8;
            var type = mavPacket.substr(base_offset, 2).toLowerCase();
            base_offset += 2;
            var autopilot = mavPacket.substr(base_offset, 2).toLowerCase();
            base_offset += 2;
            var base_mode = mavPacket.substr(base_offset, 2).toLowerCase();
            base_offset += 2;
            var system_status = mavPacket.substr(base_offset, 2).toLowerCase();
            base_offset += 2;
            var mavlink_version = mavPacket.substr(base_offset, 2).toLowerCase();

            fc.heartbeat.type = Buffer.from(type, 'hex').readUInt8(0);
            fc.heartbeat.autopilot = Buffer.from(autopilot, 'hex').readUInt8(0);
            fc.heartbeat.base_mode = Buffer.from(base_mode, 'hex').readUInt8(0);
            fc.heartbeat.custom_mode = Buffer.from(custom_mode, 'hex').readUInt32LE(0);
            fc.heartbeat.system_status = Buffer.from(system_status, 'hex').readUInt8(0);
            fc.heartbeat.mavlink_version = Buffer.from(mavlink_version, 'hex').readUInt8(0);

            muv_mqtt_client.publish(muv_pub_fc_hb_topic, JSON.stringify(fc.heartbeat));

            // TODO: disarmed에도 sortie 생성하는 문제 수정
            // if ((fc.heartbeat.base_mode & 0x80) === 0x80) {
            //         start_arm_time = moment();
            //         my_sortie_name = moment().format('YYYY_MM_DD_T_HH_mm');
            //         my_cnt_name = my_parent_cnt_name + '/' + my_sortie_name;
            //         sh_adn.crtct(my_parent_cnt_name + '?rcn=0', my_sortie_name, 0, function (rsc, res_body, count) {
            //         });
            //         cal_flag = 1;
            //         cal_sortiename = my_sortie_name;
            // } else {
            //     if (cal_flag == 1) {
            //         cal_flag = 0;
            //         calculateFlightTime(cal_sortiename);
            //     }
            //     my_sortie_name = 'disarm';
            //     my_cnt_name = my_parent_cnt_name + '/' + my_sortie_name;
            //     my_gimbal_name = my_gimbal_parent + '/' + my_sortie_name;
            // }
            if (fc.heartbeat.base_mode & 0x80) {
                if (flag_base_mode == 3) {
                    start_arm_time = moment();
                    flag_base_mode++;
                    my_sortie_name = moment().format('YYYY_MM_DD_T_HH_mm');
                    my_cnt_name = my_parent_cnt_name + '/' + my_sortie_name;
                    sh_adn.crtct(my_parent_cnt_name + '?rcn=0', my_sortie_name, 0, function (rsc, res_body, count) {
                    });

                    // for (var idx in mission_parent) {
                    //     if (mission_parent.hasOwnProperty(idx)) {
                    //         setTimeout(createMissionContainer, 10, idx);
                    //     }
                    // }
                } else {
                    flag_base_mode++;
                    if (flag_base_mode > 16) {
                        flag_base_mode = 16;
                    }
                }
            } else {
                flag_base_mode = 0;

                my_sortie_name = 'disarm';
                my_cnt_name = my_parent_cnt_name + '/' + my_sortie_name;
                my_gimbal_name = my_gimbal_parent + '/' + my_sortie_name;
            }
        } else if (msg_id == mavlink.MAVLINK_MSG_ID_SYSTEM_TIME) { // #02 : SYSTEM_TIME
            muv_mqtt_client.publish(muv_pub_fc_system_time_topic, mavPacket);
        } else if (msg_id == mavlink.MAVLINK_MSG_ID_TIMESYNC) { // #111 : TIMESYNC
            muv_mqtt_client.publish(muv_pub_fc_timesync_topic, mavPacket);
        } else if (msg_id == mavlink.MAVLINK_MSG_ID_PARAM_VALUE) {
            let param_value = mavPacket.substr(base_offset, 8).toLowerCase();
            base_offset += 8;
            let param_count = mavPacket.substr(base_offset, 4).toLowerCase();
            base_offset += 4;
            let param_index = mavPacket.substr(base_offset, 4).toLowerCase();
            base_offset += 4;
            let param_id = mavPacket.substr(base_offset, 32).toLowerCase();
            base_offset += 32;
            let param_type = mavPacket.substr(base_offset, 2).toLowerCase();

            fc.wp_yaw_behavior.id = Buffer.from(param_id, "hex").toString('ASCII').toLowerCase();
            fc.wp_yaw_behavior.id = fc.wp_yaw_behavior.id.replace(/\0/g, '');

            if (fc.wp_yaw_behavior.id === 'wp_yaw_behavior') {
                fc.wp_yaw_behavior.value = Buffer.from(param_value, 'hex').readFloatLE(0);
                fc.wp_yaw_behavior.type = Buffer.from(param_type, 'hex').readInt8(0);
                fc.wp_yaw_behavior.count = Buffer.from(param_count, 'hex').readInt16LE(0);
                fc.wp_yaw_behavior.index = Buffer.from(param_index, 'hex').readUInt16LE(0);

                muv_mqtt_client.publish(muv_pub_fc_wp_yaw_behavior_topic, JSON.stringify(fc.wp_yaw_behavior));
            }
        }
    } catch (e) {
        console.log('[parseMavFromDrone Error]', e);
    }
}


/* USER CODE */
// for sensor
let tas = {
    client: {
        connected: false,
    },

    connection: {
        host: '192.168.50.58',
        port: 1883,
        endpoint: '',
        clean: true,
        connectTimeout: 4000,
        reconnectPeriod: 4000,
        clientId: 'tas_' + nanoid(15),
        username: 'keti_thyme',
        password: 'keti_thyme',
    },
};

let DHT_sensor = {
    type: 11,
    GPIO_PIN: 4
}

let sendDataTopic = {
    temp: '/thyme/temp',
    humi: '/thyme/humi'
    // co2: '/thyme/co2',
    // tvoc: '/thyme/tvoc',
    // temp: '/thyme/temp',
};

let recvDataTopic = {
    led: '/led/set',
};
/* */

let createConnection = () => {
    if (tas.client.connected) {
        console.log('Already connected --> destroyConnection')
        destroyConnection();
    }

    if (!tas.client.connected) {
        tas.client.loading = true;
        const {host, port, endpoint, ...options} = tas.connection;
        const connectUrl = `mqtt://${host}:${port}${endpoint}`
        try {
            tas.client = mqtt.connect(connectUrl, options);

            tas.client.on('connect', () => {
                console.log(host, 'Connection succeeded!');

                tas.client.connected = true;
                tas.client.loading = false;

                // for(let topicName in recvDataTopic) {
                //     if(recvDataTopic.hasOwnProperty(topicName)) {
                //         doSubscribe(recvDataTopic[topicName]);
                //     }
                // }
            });

            tas.client.on('error', (error) => {
                console.log('Connection failed', error);

                destroyConnection();
            });

            tas.client.on('close', () => {
                console.log('Connection closed');

                destroyConnection();
            });

            tas.client.on('message', (topic, message) => {
                let content = null;
                /* USER CODES */
                // if(topic === recvDataTopic.led) {
                //     // LED 제어
                // }
                /* */
            });
        }
        catch (error) {
            console.log('mqtt.connect error', error);
            tas.client.connected = false;
        }
    }
};

let doSubscribe = (topic) => {
    if (tas.client.connected) {
        const qos = 0;
        tas.client.subscribe(topic, {qos}, (error) => {
            if (error) {
                console.log('Subscribe to topics error', error)
                return;
            }

            console.log('Subscribe to topics (', topic, ')');
        });
    }
};

let doUnSubscribe = (topic) => {
    if (tas.client.connected) {
        tas.client.unsubscribe(topic, error => {
            if (error) {
                console.log('Unsubscribe error', error)
            }

            console.log('Unsubscribe to topics (', topic, ')');
        });
    }
};

let doPublish = (topic, payload) => {
    if (tas.client.connected) {
        tas.client.publish(topic, payload, 0, error => {
            if (error) {
                console.log('Publish error', error)
            }
        });
    }
};

let destroyConnection = () => {
    if (tas.client.connected) {
        try {
            if(Object.hasOwnProperty.call(tas.client, '__ob__')) {
                tas.client.end();
            }
            tas.client = {
                connected: false,
                loading: false
            }
            console.log(this.name, 'Successfully disconnected!');
        }
        catch (error) {
            console.log('Disconnect failed', error.toString())
        }
    }
};

createConnection();

/* USER CODE */
/* DHT sensing interval */
setInterval(() => {
    sensor.read(DHT_sensor["type"], DHT_sensor["GPIO_PIN"], function(err, temperature, humidity) {
        if(!err) {
            let con_body = {
                "temp": temperature,
                "humidity": humidity
            }
            console.log(`DHT sensing result [temp: ${temperature}°C, humidity: ${humidity}%]`);
            doPublish(sendDataTopic['temp'], temperature.toString());
            doPublish(sendDataTopic['humi'], humidity.toString());
        }
        else {
            console.log("DHT_sensing err :", err);
        }
    });
}, 3000);

/* */
