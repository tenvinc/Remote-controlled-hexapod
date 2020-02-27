const express = require('express');
const app = express();
const path = require('path');
const Serial = require('serialport');
const port = new Serial('/dev/ttyACM0', {
    baudRate: 115200,
    autoOpen: false,
    dataBits: 8,
    parity: 'none',
    stopBits: 1
});
const Readline = require('@serialport/parser-readline');
const parser = port.pipe(new Readline({ delimiter: '\n' }))
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


var json2 = {
    'containReading': false,
    'message': "This represents a debug message."
}

var periodicMessage = setInterval(() => {
    clients.forEach(client => {
        client.send(JSON.stringify(json2));
    });
}, 20);

port.open((err) => {
    if (err) console.log("Error has occured. " + err.message);
});

port.on('open', () => {
    console.log("Connection with Arduino established.")
});

parser.on('data', (data) => {
    const re = /Data: ([0-9., -]+)/;
    
    let matcher = data.match(re);
    if (matcher === null) return;
    
    let filt = matcher[1];
    var params = filt.split(",");
    params = params.map(item => { return parseFloat(item); });
    console.log(params);
    
    let json = {
        'containReading': true,
        'roll': params[0],
        'pitch': params[1],
        'yaw': 0
    }

    clients.forEach(client => {
        if (client.readyState == WebSocket.OPEN) {
            client.send(JSON.stringify(json));
        }
    });
});

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