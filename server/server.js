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

const wsServer = new WebSocket.Server({ port: 8080 })

var clients = [];

wsServer.on('connection', ws => {
    var index = clients.push(ws) - 1;
    ws.on('message', (message) => {
        if (message.type == 'utf-8') {
            console.log(message);
        }
    });
    ws.send("thank you for the connection!");
});

var cnt = 0;
var json = {
    'containReading': true,
    'roll': 40.0,
    'pitch': 50.0,
    'yaw': 60.0 
}
setInterval(() => {
    clients.forEach(client => {
        client.send(JSON.stringify(json));
    });
    json.roll += 0.1;
    json.pitch += 0.1;
    json.yaw += 0.1;
}, 100);

var json2 = {
    'containReading': false,
    'message': "This represents a debug message."
}

var periodicMessage = setInterval(() => {
    clients.forEach(client => {
        client.send(JSON.stringify(json2));
    });
}, 20);

// port.open((err) => {
//     if (err) console.log("Error has occured. " + err.message);
// });

// port.on('open', () => {
//     console.log("Connection with Arduino established.")
// });

// parser.on('data', (data) => {
//     clients.forEach(client => {
//         if (client.readyState == WebSocket.OPEN) {
//             console.log("Data: " + data);
//             client.send(data);
//         }
//     });
// });

app.use(express.static(__dirname + '/public'));

app.get('/', (req, res) => {
    if (req.method == "GET") {
        res.sendFile(path.join(__dirname + '/public/app.html'), (err) => {
            if (err) console.error("Error has occured!" + err);
        });
    }
});

app.listen(5000, () => {
    console.log("HTTP server listening on port 5000.");
});