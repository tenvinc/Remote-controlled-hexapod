/************************** Constants declaration ************************************/
const MAX_MESSAGES_CNT = 50;
const SERVER_URI = "ws://localhost:8080";
const DEFAULT_CAM_POS = 2;

/************************** 3D modelling stuff ***************************************/
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight,
    0.1, 1000);

const renderer = new THREE.WebGLRenderer({ antialias: true });
const vizBody = document.getElementById("viz-body");
const geometry = new THREE.BoxGeometry(2, 0.2, 1);
const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
const cube = new THREE.Mesh(geometry, material);
scene.add(cube);

var nextRoll = 0;
var nextPitch = 0;
var nextYaw = 0;

// Initialize z to move camera out of model
camera.position.z = DEFAULT_CAM_POS;

function createElementFromHtml(html) {
    let template = document.createElement('template');
    console.log(template);
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
    let rollDegrees = (rotation.z / 2 * Math.PI) * 360;
    let pitchDegrees = (rotation.x / 2 * Math.PI) * 360;
    let yawDegrees = (rotation.y / 2 * Math.PI) * 360;

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

function animate() {
    requestAnimationFrame(animate);
    // Javascript frame reference: Roll: -z, Pitch: x, Yaw: -y
    cube.rotation.x = nextPitch;
    cube.rotation.y = -nextYaw;
    cube.rotation.z = -nextRoll;
    updateHud(cube.rotation);
    renderer.render(scene, camera);
}

function resizeViz() {
    const displayWidth = renderer.domElement.clientHeight;
    const displayHeight = renderer.domElement.clientWidth;
    let canvas = renderer.domElement;

    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();

    if (canvas.height !== displayHeight || canvas.width !== displayWidth) {
        renderer.setSize(displayWidth, displayHeight, false);
        console.log("Canvas resized to " + canvas.height + ", " + canvas.width);
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

/*********************************************************************/

var terminal = document.querySelector("#debug-terminal");
connectBtn.activateListener(setupConnection, teardownConnection);
window.addEventListener('resize', resizeViz, false)
animate();
resizeViz();


