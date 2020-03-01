/************************** Constants declaration ************************************/
const MAX_MESSAGES_CNT = 50;
const SERVER_URI = "ws://localhost:8080";
const DEFAULT_CAM_POS = 20;

/************************** 3D modelling stuff ***************************************/
const loader = new THREE.GLTFLoader();
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight,
    0.1, 1000);

var light = new THREE.PointLight(0xc4c4c4, 5);
light.position.set(0, 300, 500);
scene.add(light);

var light2 = new THREE.PointLight(0xc4c4c4, 5);
light2.position.set(500, 100, 0);
scene.add(light2);

var light3 = new THREE.PointLight(0xc4c4c4, 5);
light3.position.set(0, 100, -500);
scene.add(light3);

var light4 = new THREE.PointLight(0xc4c4c4, 5);
light4.position.set(-500, 300, 0);
scene.add(light4);

const renderer = new THREE.WebGLRenderer({ antialias: true });
const vizBody = document.getElementById("viz-body");
var model;
loader.load('./robot_hand.gltf', function(gltf) {
    model = gltf.scene;
    model.scale.set(0.1, 0.1, 0.1);
    console.log(gltf.scene);
    scene.add(model);
    animate();
},
// called while loading is progressing
function ( xhr ) {
    console.log( ( xhr.loaded / xhr.total * 100 ) + '% loaded' );
},
// called when loading has errors
function ( error ) {
    console.log( 'An error happened: ' + error);
});

var nextRoll = 0;
var nextPitch = 0;
var nextYaw = 0;

// Initialize z to move camera out of model
camera.position.z = DEFAULT_CAM_POS;

function createElementFromHtml(html) {
    let template = document.createElement('template');
    html = html.trim();
    template.innerHTML = html;
    return template.content.cloneNode(true);
}

const hudHtml = `<div class="viz-hud">
<div class="label">Roll (deg): </div>
<div id="hud-roll" class="element">30.0</div>
<div class="label" >Pitch (deg): </div>
<div id="hud-pitch" class="element">40.0</div>
<div class="label">Yaw (deg): </div>
<div id="hud-yaw" class="element">50.0</div>
</div>`;

var hud = createElementFromHtml(hudHtml);

renderer.domElement.classList.add("viz-canvas");
renderer.domElement.removeAttribute("style");
vizBody.appendChild(renderer.domElement);
vizBody.appendChild(hud);

// DOM elements
var hudRoll = vizBody.querySelector("#hud-roll");
var hudPitch = vizBody.querySelector("#hud-pitch");
var hudYaw = vizBody.querySelector("#hud-yaw");

function updateHud(rotation) {
    // Javascript frame reference: Roll: z, Pitch: x, Yaw: y
    let rollDegrees = (rotation.z / (2 * Math.PI)) * 360;
    let pitchDegrees = (rotation.x / (2 * Math.PI)) * 360;
    let yawDegrees = (rotation.y / (2 * Math.PI)) * 360;

    // Restrict range to [0, 360]
    rollDegrees = rollDegrees - Math.floor(rollDegrees / 360) * 360;
    pitchDegrees = pitchDegrees - Math.floor(pitchDegrees / 360) * 360;
    yawDegrees = yawDegrees - Math.floor(yawDegrees / 360) * 360;

    // Restrict range to [-180, 180]
    rollDegrees = (rollDegrees <= 180) ? rollDegrees : -(360 - rollDegrees);
    pitchDegrees = (pitchDegrees <= 180) ? pitchDegrees : -(360 - pitchDegrees);
    yawDegrees = (yawDegrees <= 180) ? yawDegrees : -(360 - yawDegrees);

    // Transform to real world representation 
    rollDegrees = -rollDegrees;  // Roll left is -ve instead of javascript +ve
    yawDegrees = -yawDegrees;  // Yaw left is -ve instead of javascript +ve

    hudRoll.textContent = rollDegrees.toFixed(2);
    hudPitch.textContent = pitchDegrees.toFixed(2);
    hudYaw.textContent = yawDegrees.toFixed(2);
}

function resizeViz() {
    const displayWidth = renderer.domElement.clientHeight;
    const displayHeight = renderer.domElement.clientWidth;
    let canvas = renderer.domElement;

    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();

    if (canvas.height !== displayHeight || canvas.width !== displayWidth) {
        renderer.setSize(displayWidth, displayHeight, false);
        console.log("3D Canvas resized to " + canvas.height + ", " + canvas.width);
    }
}

/********************** Connection button toggle **********************/
var ToggleButton = function (element) {
    this.DEFAULT = {
        on: 'On',
        off: 'Off',
        onstyle: 'primary',
        offstyle: 'light',
        size: 'normal',
        style: '',
        width: null,
        height: null
    };
    this.element = element;
    this.toggleOn = null;
    this.toggleOff = null;
    this.wrapper = null;
    this.on = this.DEFAULT.on;
    this.off = this.DEFAULT.off;
    this.onstyle = element.getAttribute("data-onstyle") ?
        "btn-" + element.getAttribute("data-onstyle") : "btn-" + this.DEFAULT.onstyle;
    this.offstyle = element.getAttribute("data-offstyle") ?
        "btn-" + element.getAttribute("data-offstyle") : "btn-" + this.DEFAULT.offstyle;
    this.render();
    this.activateListener = function (setupCB, teardownCB) {
        let btn = this;
        this.wrapper.addEventListener('click', function (event) {
            btn.toggle(setupCB, teardownCB);
        });
    }
}

ToggleButton.prototype.render = function () {
    parent = this.element.parentElement;
    this.wrapper = document.createElement("div");
    this.wrapper.classList.add("toggle", "btn", this.offstyle, "off");
    this.wrapper.setAttribute("role", "button");
    this.wrapper.style = "width: 60.3438px; height: 38px;"

    parent.insertBefore(this.wrapper, this.element);
    this.wrapper.appendChild(this.element);

    let toggleGroup = document.createElement("div");
    toggleGroup.classList.add("toggle-group");
    this.wrapper.appendChild(toggleGroup);

    this.toggleOn = document.createElement("label");
    this.toggleOn.textContent = this.on;
    this.toggleOn.classList.add("btn", this.onstyle, "toggle-on");
    this.toggleOff = document.createElement("label");
    this.toggleOff.textContent = this.off;
    this.toggleOff.classList.add("btn", this.offstyle, "toggle-off");
    let span = document.createElement("span");
    span.classList.add('toggle-handle', "btn-light");

    toggleGroup.appendChild(this.toggleOn);
    toggleGroup.appendChild(this.toggleOff);
    toggleGroup.appendChild(span);
}

ToggleButton.prototype.toggle = function (setupCB, teardownCB) {
    if (this.element.checked) {
        // this.turnOff(callback);
        teardownCB(this);
    } else {
        // this.turnOn(callback);
        setupCB(this);
    }
}

ToggleButton.prototype.turnOn = function () {
    console.log("Turning on the connection");
    this.wrapper.classList.remove(this.offstyle, "off");
    this.wrapper.classList.add(this.onstyle);
    this.element.checked = true;
}

ToggleButton.prototype.turnOff = function () {
    console.log("Turning off the connection");
    this.wrapper.classList.remove(this.onstyle);
    this.wrapper.classList.add(this.offstyle, "off");
    this.element.checked = false;
}

var toggle = document.querySelector("input[class=toggle][type=checkbox]");
var connectBtn = new ToggleButton(toggle);

/************************ Websockets *************************************/
var socket;

function setupConnection(context) {
    socket = new WebSocket("ws://localhost:8080");

    socket.onopen = function (e) {
        alert("[open] Connection established");
        alert("Sending to server");
        context.turnOn();
    };

    socket.onmessage = function (event) {
        try {
            let json = JSON.parse(event.data);
            console.log(json);
            if (json.containReading) {
                // reading in degrees, convert to radians
                nextRoll = json.roll * 2 * Math.PI / 360;
                nextPitch = json.pitch * 2 * Math.PI / 360;
                nextYaw = json.yaw * 2 * Math.PI / 360;

                // convert time into seconds
                time = json.time / 1000;
                rollReadings.push(new Point2D(time, nextRoll));
                pitchReadings.push(new Point2D(time, nextPitch));

            } else if ("message" in json) {
                while (terminal.children.length > MAX_MESSAGES_CNT) {
                    console.log("Deleting previous debug messages starting from oldest...");
                    terminal.removeChild(terminal.firstElementChild);
                }
                let message = createElementFromHtml(`<div class="message">${json.message}</div>`);
                terminal.appendChild(message);
            }
        } catch (err) {
            if (err instanceof SyntaxError) {
                console.log("Error in json string. Message will be ignored.")
            } else {
                throw err;
            }
        }
    };

    socket.onclose = function (event) {
        if (event.wasClean) {
            alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
        } else {
            // e.g. server process killed or network down
            // event.code is usually 1006 in this case
            alert('[close] Connection died');
        }
        context.turnOff();
    };

    socket.onerror = function (error) {
        alert(`[error] ${error.message}`);
    };
}

function teardownConnection() {
    socket.close();
    socket = undefined;
}

/****************************** Real time graphs **********************************/
const Point2D = function (x, y) {
    this.x = x;
    this.y = y;
}

var GraphOptions = function (options) {
    // Range of axis
    if (options === undefined) {
        options = {};
    }
    this.minX = (options.minX !== undefined) ? options.minX : -10;
    this.maxX = (options.maxX !== undefined) ? options.maxX : 10;
    this.minY = (options.minY !== undefined) ? options.minY : -2;
    this.maxY = (options.maxY !== undefined) ? options.maxY : 2;
    let originX = (options.originX !== undefined) ? options.originX : 0;
    let originY = (options.originY !== undefined) ? options.originY : 0;
    this.origin = new Point2D(originX, originY);

    this.xTick = (options.xTick !== undefined) ? options.xTick : (this.maxX - this.minX) / 10;
    this.yTick = (options.yTick !== undefined) ? options.yTick : (this.maxY - this.minY) / 10;

    this.tickColor = (options.tickColor !== undefined) ? options.tickColor : 'white';
    this.tickSize = (options.tickSize !== undefined) ? options.tickSize : 10;
    this.tickWidth = (options.tickWidth !== undefined) ? options.tickWidth : 3;

    this.crossSize = (options.crossSize !== undefined) ? options.crossSize : 2;

    this.arrowColor = (options.arrowColor !== undefined) ? options.arrowColor : 'white';
    this.arrowLength = (options.arrowLength !== undefined) ? options.arrowLength : 10;
    this.arrowWidth = (options.arrowWidth !== undefined) ? options.arrowWidth : 3;

    this.axisColor = (options.axisColor !== undefined) ? options.axisColor : 'white';
    this.axisWidth = (options.axisWidth !== undefined) ? options.axisWidth : 3;

    this.textFont = (options.textFont !== undefined) ? options.textFont : '10px sans-serif';
    this.textColor = (options.textColor !== undefined) ? options.textColor : 'white';
}

var Plot = function (params, options, canvas) {
    if (!(options instanceof GraphOptions)) {
        throw "Please supply a GraphOptions object";
    }
    this.options = options;

    // padding measured in pixels
    this.paddingLeft = (params.paddingLeft) ? params.paddingLeft : 30;
    this.paddingRight = (params.paddingRight) ? params.paddingRight : 30;
    this.paddingTop = (params.paddingTop) ? params.paddingTop : 10;
    this.paddingBtm = (params.paddingBtm) ? params.paddingBtm : 10;

    this.graphWidth = canvas.width - this.paddingLeft - this.paddingRight;
    this.graphHeight = canvas.height - this.paddingTop - this.paddingBtm;

    this.topLeft = new Point2D(this.paddingLeft, this.paddingTop);
    this.topRight = new Point2D(this.paddingLeft + this.graphWidth, this.paddingTop);
    this.btmLeft = new Point2D(this.paddingLeft, this.paddingTop + this.graphHeight);
    this.btmRight = new Point2D(this.paddingLeft + this.graphWidth, this.paddingTop + this.graphHeight);

    this.ctx = canvas.getContext('2d');
}

Plot.prototype.transformToScreenFrame = function (point) {
    let x = this.btmLeft.x + ((point.x - this.options.minX) / (this.options.maxX - this.options.minX)) * this.graphWidth;
    let y = this.btmLeft.y - ((point.y - this.options.minY) / (this.options.maxY - this.options.minY)) * this.graphHeight;
    return new Point2D(x, y);
}

Plot.prototype.drawAxis = function () {
    var originScreen = this.transformToScreenFrame(this.options.origin);

    var xAxisStart = new Point2D(this.btmLeft.x, originScreen.y);
    var xAxisEnd = new Point2D(this.btmRight.x, originScreen.y);
    var yAxisStart = new Point2D(originScreen.x, this.btmLeft.y);
    var yAxisEnd = new Point2D(originScreen.x, this.topLeft.y);

    // Draw the lines
    this.ctx.lineWidth = this.options.axisWidth;
    this.ctx.strokeStyle = this.options.axisColor;

    this.ctx.beginPath();
    this.ctx.moveTo(xAxisStart.x, xAxisStart.y);
    this.ctx.lineTo(xAxisEnd.x, xAxisEnd.y);

    this.ctx.moveTo(yAxisStart.x, yAxisStart.y);
    this.ctx.lineTo(yAxisEnd.x, yAxisEnd.y);

    this.ctx.stroke();
    // Draw the arrows
    this.ctx.lineWidth = this.options.arrowWidth;
    this.ctx.strokeStyle = this.options.arrowColor;

    this.ctx.beginPath()
    this.ctx.moveTo(xAxisEnd.x, xAxisEnd.y);
    this.ctx.lineTo(xAxisEnd.x - this.options.arrowLength * Math.cos(Math.PI / 4),
        xAxisEnd.y - this.options.arrowLength * Math.sin(Math.PI / 4));
    this.ctx.moveTo(xAxisEnd.x, xAxisEnd.y);
    this.ctx.lineTo(xAxisEnd.x - this.options.arrowLength * Math.cos(Math.PI / 4),
        xAxisEnd.y + this.options.arrowLength * Math.sin(Math.PI / 4));

    this.ctx.moveTo(yAxisEnd.x, yAxisEnd.y);
    this.ctx.lineTo(yAxisEnd.x - this.options.arrowLength * Math.sin(Math.PI / 4),
        yAxisEnd.y + this.options.arrowLength * Math.cos(Math.PI / 4));
    this.ctx.moveTo(yAxisEnd.x, yAxisEnd.y);
    this.ctx.lineTo(yAxisEnd.x + this.options.arrowLength * Math.sin(Math.PI / 4),
        yAxisEnd.y + this.options.arrowLength * Math.cos(Math.PI / 4));

    this.ctx.stroke();
}

Plot.prototype.drawMajorTicks = function () {
    let xGap = (this.options.minX - this.options.origin.x) / this.options.xTick;
    if (xGap < 0) {
        xGap = -(Math.floor(Math.abs(xGap)));
    } else {
        xGap = Math.floor(xGap);
    }
    let yGap = (this.options.minY - this.options.origin.y) / this.options.yTick;
    if (yGap < 0) {
        yGap = -(Math.floor(Math.abs(yGap)));
    } else {
        yGap = Math.floor(yGap);
    }
    let posX = this.options.origin.x + xGap * this.options.xTick;
    let posY = this.options.origin.y + yGap * this.options.yTick;
    this.ctx.lineWidth = this.options.tickWidth;
    this.ctx.strokeStyle = this.options.tickColor;
    this.ctx.beginPath();
    while (posX < this.options.maxX) {
        let screenPoint = this.transformToScreenFrame(new Point2D(posX, this.options.origin.y));
        this.ctx.moveTo(screenPoint.x, screenPoint.y + this.options.tickSize / 2);
        this.ctx.lineTo(screenPoint.x, screenPoint.y - this.options.tickSize / 2);
        this.ctx.font = this.options.textFont;
        this.ctx.fillStyle = this.options.textColor;
        this.ctx.fillText(posX.toFixed(2), screenPoint.x, screenPoint.y + this.options.tickSize/2 + 10);
        posX += this.options.xTick;
    }
    while (posY < this.options.maxY) {
        let screenPoint = this.transformToScreenFrame(new Point2D(this.options.origin.x, posY));
        this.ctx.moveTo(screenPoint.x + this.options.tickSize / 2, screenPoint.y);
        this.ctx.lineTo(screenPoint.x - this.options.tickSize / 2, screenPoint.y);
        this.ctx.font = this.options.textFont;
        this.ctx.fillStyle = this.options.textColor;
        this.ctx.fillText(posY.toFixed(2), screenPoint.x - this.options.tickSize/2 - 25, screenPoint.y);
        posY += this.options.yTick;
    }
    this.ctx.stroke();
}

Plot.prototype.drawCross = function (point, crossColor, crossWidth) {
    let screenPoint = this.transformToScreenFrame(point);
    // console.log(screenPoint);
    this.ctx.lineWidth = crossWidth;
    this.ctx.strokeStyle = crossColor;

    this.ctx.beginPath();
    this.ctx.moveTo(screenPoint.x - this.options.crossSize / 2, screenPoint.y - this.options.crossSize / 2);
    this.ctx.lineTo(screenPoint.x + this.options.crossSize / 2, screenPoint.y + this.options.crossSize / 2);
    this.ctx.moveTo(screenPoint.x - this.options.crossSize / 2, screenPoint.y + this.options.crossSize / 2);
    this.ctx.lineTo(screenPoint.x + this.options.crossSize / 2, screenPoint.y - this.options.crossSize / 2);
    this.ctx.stroke();
}

Plot.prototype.draw = function (points, crossColor, crossWidth) {
    for (let i = 0; i < points.length; i++) {
        let point = points[i];
        // console.log(point);
        this.drawCross(point);
    }
}

Plot.prototype.updateCanvasSize = function () {
    canvas = this.ctx.canvas;
    this.graphWidth = canvas.width - this.paddingLeft - this.paddingRight;
    this.graphHeight = canvas.height - this.paddingTop - this.paddingBtm;

    this.topLeft = new Point2D(this.paddingLeft, this.paddingTop);
    this.topRight = new Point2D(this.paddingLeft + this.graphWidth, this.paddingTop);
    this.btmLeft = new Point2D(this.paddingLeft, this.paddingTop + this.graphHeight);
    this.btmRight = new Point2D(this.paddingLeft + this.graphWidth, this.paddingTop + this.graphHeight);
}

/************************************************************************/
var terminal = document.querySelector("#debug-terminal");
connectBtn.activateListener(setupConnection, teardownConnection);

function initCanvas(canvas) {
    // Do twice to cater to the flex grow factor
    let width = canvas.parentElement.clientWidth;
    let height = canvas.parentElement.clientHeight;
    canvas.setAttribute('width', width);
    canvas.setAttribute('height', height);
    width = canvas.parentElement.clientWidth;
    height = canvas.parentElement.clientHeight;
    canvas.setAttribute('width', width);
    canvas.setAttribute('height', height);
    return canvas;
}

var graphRoll = initCanvas(document.querySelector("#roll-graph"));
var graphPitch = initCanvas(document.querySelector("#pitch-graph"));
var graphYaw = initCanvas(document.querySelector("#yaw-graph"));
var config = new GraphOptions({'minX': 0, 'maxX': 5, 'minY': -3.14, 'maxY': 3.14, 'yTick': 0.5, 'crossSize': 0.5});
var rollPlot = new Plot({}, config, graphRoll);
var pitchPlot = new Plot({}, config, graphPitch);
var yawPlot = new Plot({}, config, graphYaw);

var rollReadings = [];
var pitchReadings = [];
var yawReadings = [];

function resizeGraphs() {
    plots = [rollPlot, pitchPlot, yawPlot];
    for (let i = 0; i < plots.length; i++) {
        let canvas = plots[i].ctx.canvas;
        initCanvas(canvas);
        plots[i].updateCanvasSize();
    }
}

var x = rollPlot.options.minX;
var points = [];
console.log(points);
while (x < rollPlot.options.maxX) {
    points.push(new Point2D(x, Math.sin(x)));
    x += 0.1;
}

function redrawGraph(points, plot) {
    if (points.length > 0 && points[points.length - 1].x > plot.options.maxX) {
        console.log("New points out of range. Translating graph by removing old points...");
        let newMaxX = points[points.length - 1].x + 0.1;
        let diff = newMaxX - plot.options.maxX;
        let newMinX = plot.options.minX + diff;
        let cnt = 0;
        for (let i = 0; i < points.length; i++) {
            if (points[i].x < newMinX) cnt++;
            break;
        }
        points.splice(0, cnt);
        plot.options.minX = newMinX;
        plot.options.maxX = newMaxX;
    }

    plot.ctx.clearRect(0, 0, plot.ctx.canvas.width, plot.ctx.canvas.height);
    plot.drawAxis();
    plot.drawMajorTicks();
    plot.draw(points);
}

function animate() {
    requestAnimationFrame(animate);
    // Javascript frame reference: Roll: -z, Pitch: x, Yaw: -y
    model.rotation.x = nextPitch;
    model.rotation.y = -nextYaw;
    model.rotation.z = -nextRoll;
    updateHud(model.rotation);
    renderer.render(scene, camera);

    // Update real time graph
    redrawGraph(rollReadings, rollPlot);
    redrawGraph(pitchReadings, pitchPlot);
    redrawGraph(yawReadings, yawPlot);
}

window.addEventListener('resize', () => {
    console.log("Resizing everything");
    resizeViz();
    resizeGraphs();
}, false);

resizeViz();