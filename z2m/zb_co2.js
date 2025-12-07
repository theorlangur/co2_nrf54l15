const {co2,battery,temperature,humidity} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['CO2-54L15-NG'],
    model: 'CO2-54L15-NG',
    fingerprint: [{modelID: 'CO2-NG', applicationVersion: 1, priority: -1},],
    vendor: 'SFINAE',
    description: 'CO2 Sensor',
    extend: [
        battery({
            voltage: true, 
            voltageReporting: true
        }),
        co2(),
        temperature(),
        humidity()
    ],
    configure: async (device, coordinatorEndpoint) => {
        //dummy
    },

};

module.exports = definition;
