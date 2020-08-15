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
    socket = new WebSocket("ws://localhost:8081");

    socket.onopen = function (e) {
        alert("[open] Connection established");
        alert("Sending to server");
        context.turnOn();
        socket.isConnected = true;
    };

    socket.onmessage = (event) => {
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

/************************************************************************/
connectBtn.activateListener(setupConnection, teardownConnection);

const dpads = ["d-pad-left", "d-pad-right", "d-pad-up", "d-pad-down", "d-pad-turn-left", "d-pad-turn-right"];
const dpadElements = {};
const commands = {
  "d-pad-left": "LEFT",
  "d-pad-right": "RIGHT",
  "d-pad-up": "FORWARD",
  "d-pad-down": "BACKWARD",
  "d-pad-turn-left": "STAND",
  "d-pad-turn-right": "KEEP"
};
const dpadIntervals = {};

dpads.forEach(dpad => {
  dpadElements[dpad] = document.getElementById(dpad);
  dpadElements[dpad].addEventListener('mousedown', () => {
    if (typeof socket !== 'undefined' && socket.isConnected) {
      socket.send(commands[dpad]);
    }
    dpadIntervals[dpad] = setInterval(() => {
      if (typeof socket !== 'undefined' && socket.isConnected) {
        socket.send(commands[dpad]);
      }
    }, 200);
  });
  dpadElements[dpad].addEventListener('mouseup', () => {
    if (dpadIntervals[dpad]) {
      clearInterval(dpadIntervals[dpad]);
      delete dpadIntervals[dpad];
    }
  });
});
