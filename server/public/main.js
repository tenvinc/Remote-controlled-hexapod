const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight,
    0.1, 1000);

const renderer = new THREE.WebGLRenderer({ antialias: true });
const vizBody = document.getElementById("viz-body");

const geometry = new THREE.BoxGeometry();
const material = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
const cube = new THREE.Mesh( geometry, material );
scene.add( cube );

camera.position.z = 2;
console.log(camera.position)

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

var hudRoll = vizBody.querySelector("#hud-roll");
var hudPitch = vizBody.querySelector("#hud-pitch");
var hudYaw = vizBody.querySelector("#hud-yaw");

var nextRoll = 0;
var nextPitch = 0;
var nextYaw = 0;

function updateHud(rotation) {
    let rollDegrees = (rotation.x / Math.PI) * 360;
    let pitchDegrees = (rotation.y / Math.PI) * 360;
    let yawDegrees = (rotation.z / Math.PI) * 360;

    // Restrict range to [0, 360]
    rollDegrees = rollDegrees - Math.floor(rollDegrees / 360) * 360;
    pitchDegrees = pitchDegrees - Math.floor(pitchDegrees / 360) * 360;
    yawDegrees = yawDegrees - Math.floor(yawDegrees / 360) * 360;

    hudRoll.textContent = rollDegrees.toFixed(2);
    hudPitch.textContent = pitchDegrees.toFixed(2);
    hudYaw.textContent = yawDegrees.toFixed(2);
}   

function animate() {
    requestAnimationFrame(animate);
    
    cube.rotation.x = nextPitch;
    cube.rotation.y = nextYaw;
    cube.rotation.z = nextRoll;
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

window.addEventListener('resize', resizeViz, false)

animate();
resizeViz();

const socket = new WebSocket("ws://localhost:8080");

socket.onopen = function(e) {
    alert("[open] Connection established");
    alert("Sending to server");
  };
  
socket.onmessage = function(event) {
    let json = JSON.parse(event.data);
    if (json.containReading) {
        nextRoll = json.roll;
        nextPitch = json.pitch;
        nextYaw = json.yaw;
    }
};

socket.onclose = function(event) {
if (event.wasClean) {
    alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
} else {
    // e.g. server process killed or network down
    // event.code is usually 1006 in this case
    alert('[close] Connection died');
}
};

socket.onerror = function(error) {
alert(`[error] ${error.message}`);
};

