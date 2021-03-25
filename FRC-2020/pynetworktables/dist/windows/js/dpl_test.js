function DPLConnect() {
  if (NetworkTables.isRobotConnected() ) 
  { 
    console.log("DPL Connect: Robot is connected");
    
    const keys = NetworkTables.getKeys();
    for (const k of keys) {
      if (k.includes("/Commands/")) {
        console.log(k);
      }
    }

    
  }
  else {console.log("NO ROBOT - Check that pynetworktables is running."); }
}

function logMapElements(value, key, map) {
  console.log(`m[${key}] = ${value}`);
}
