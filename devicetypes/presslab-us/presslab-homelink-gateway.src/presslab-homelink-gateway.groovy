/*
 *  3 Button Presslab Homelink Gateway
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 *  in compliance with the License. You may obtain a copy of the License at:
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
 *  on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License
 *  for the specific language governing permissions and limitations under the License.
 */
metadata {
    definition (name: "Presslab Homelink Gateway", namespace: "presslab-us", author: "Ryan Press") {
	capability "Switch"
	capability "Button"
        capability "Configuration"
        
        attribute "groupId", "Number"

        fingerprint endpointId: "01", profileId: "0104", deviceId: "0000", inClusters: "0000,0402", outClusters: "0000,0006"
    }

// simulator metadata
    simulator {
    }

    // UI tile definitions
    tiles {
	standardTile("button", "device.button", width: 2, height: 2) {
	    state "default", label: "", icon: "st.Home.home30", backgroundColor: "#ffffff"
	}
        
        main (["button"])
        details (["button"])
    }

    preferences {
	section("Switch 1 Binding") {
	    input "binding0", "text", title: "Switch 1 Binding", required: false, displayDuringSetup: true
	}
	section("Switch 2 Binding") {
	    input "binding1", "text", title: "Switch 2 Binding", required: false, displayDuringSetup: true
	}
	section("Switch 3 Binding") {
	    input "binding2", "text", title: "Switch 3 Binding", required: false, displayDuringSetup: true
	}
    }
}

def updated() {
    log.debug "updated()"
}

// Parse incoming device messages to generate events
def parse(String description) {
    log.trace "Parse description $description"
    def result = null

    if (description?.startsWith('catchall:') || description?.startsWith('read attr -')) {     
        def descMap = zigbee.parseDescriptionAsMap(description)

        if (descMap.clusterInt == 6 && descMap.command == "02") {
	    def button = descMap.sourceEndpoint.toInteger()

	    result = createEvent(name: "button", value: "pushed", data: [buttonNumber: button],
            		    descriptionText: "$device.displayName button $button is pushed", isStateChange: true)
        }
    }

    if (result != null) {
	sendEvent(result)
	log.debug "Parse returned ${result}"
    }
}

// Commands to group device
def configureGroups() {
    log.debug "Configuring groups "
    def configCmds = []
    def bindings = [binding0, binding1, binding2]
    bindings.eachWithIndex { it, idx ->
	if (it) {
    	def destList = it.tokenize(',')
	    destList.each {
		def dest = it.tokenize(':')
		def ep = dest[1].find("[0-9]+")
                // Create group ID from MAC of switch
                def group = Integer.toHexString(Integer.parseInt(device.zigbeeId[-4..-1], 16) & 0x3FFF | idx << 14)
		// Send group add command
//				configCmds << ["st cmd 0x${dest[0]} 0x${dest[1]} 0x0004 0x02 {00}","delay 1500"] // Request groups
		configCmds << ["st cmd 0x${dest[0]} 0x${dest[1]} 0x0004 0x00 {${group[2..3]} ${group[0..1]} 00}","delay 1500"]
	    }
        }
    }
    log.info "Sending ZigBee group add commands"
    log.debug configCmds
    return configCmds
}

def configure() { 
    def configCmds = [
         // Bind on/off
        "zdo bind 0x${device.deviceNetworkId} 0x01 0x01 0x0006 {${device.zigbeeId}} {}", "delay 1500",
        "zdo bind 0x${device.deviceNetworkId} 0x02 0x01 0x0006 {${device.zigbeeId}} {}", "delay 1500",
        "zdo bind 0x${device.deviceNetworkId} 0x03 0x01 0x0006 {${device.zigbeeId}} {}", "delay 1500",
    ]
    configCmds += configureGroups()
    log.info "Sending ZigBee Bind commands to device"
    log.debug configCmds
    return configCmds
}
