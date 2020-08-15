const express = require('express');
const app = express();
const path = require('path');
// const Serial = require('serialport');
// const port = new Serial('/dev/ttyACM0', {
//     baudRate: 115200,
//     autoOpen: false,
//     dataBits: 8,
//     parity: 'none',
//     stopBits: 1
// });
// const Readline = require('@serialport/parser-readline');
// const parser = port.pipe(new Readline({ delimiter: '\n' }))
const WebSocket = require('ws');

const wsDevServer = new WebSocket.Server({ port: 8080 })
const wsClientServer = new WebSocket.Server( {port: 8081 })

var devices = [];  // represents the gloves
var clients = [];  // represents client connecting through browser

var flag = false;

wsDevServer.on('connection', ws => {
    devices.push(ws);
    ws.on('message', (message) => {
        if (!flag) {
            console.log("First connection from device!");
            flag = true;
        }
        clients.forEach(client => {
            client.send(message);  // pass through
        });
    });
});

wsClientServer.on('connection', ws => {
    var index = clients.push(ws) - 1;
    ws.on('message', (message) => {
        devices.forEach(dev => {
            dev.send(message);  // pass through
        })
        console.log(message);
    });
    console.log("Client signed in.");
});

app.use(express.static(__dirname + '/public'));

app.get('/', (req, res) => {
    if (req.method == "GET") {
        res.sendFile(path.join(__dirname + '/public/glove_dashboard.html'), (err) => {
            if (err) console.error("Error has occured!" + err);
        });
    }
});

app.get('/gdashboard', (req, res) => {
    if (req.method == "GET") {
        res.sendFile(path.join(__dirname + '/public/glove_dashboard.html'), (err) => {
            if (err) console.error("Error has occured!" + err);
        });
    }
});

app.get('/manualcontrol', (req, res) => {
    if (req.method == "GET") {
        res.sendFile(path.join(__dirname + '/public/hexy_manualctrl.html'), (err) => {
            if (err) console.error("Error has occured!" + err);
        });
    }
});

app.listen(5000, () => {
    console.log("HTTP server listening on port 5000.");
});