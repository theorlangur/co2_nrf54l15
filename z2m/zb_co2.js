const {Zcl} = require('zigbee-herdsman');
const {co2,battery,temperature,humidity,deviceAddCustomCluster,setupConfigureForReading,setupConfigureForReporting} = require('zigbee-herdsman-converters/lib/modernExtend');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets;
const eo = exposes.options;
const ea = exposes.access;

const orlangurSCD4XExtended = {
    scd4x: () => {
        const exposes = [
            e.numeric('man_measure', ea.STATE_GET).withLabel('Manual Measure Count').withCategory('diagnostic'),
            e.numeric('factory_resets', ea.STATE_GET).withLabel('Factory Reset Count').withCategory('diagnostic'),
            e.numeric('scd_variant', ea.STATE_GET).withLabel('SCD4X Variant').withCategory('diagnostic'),
            e.enum("cmd_factory_reset", ea.SET, ["Factory Reset"])
                .withDescription("Factory Reset SCD4X")
                .withCategory("config"),
        ];

        const fromZigbee = [
            {
                cluster: 'customSCD4x',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    if (data['man_measure'] !== undefined) 
                        result['man_measure'] = data['man_measure'];
                    if (data['factory_resets'] !== undefined) 
                        result['factory_resets'] = data['factory_resets'];
                    if (data['scd_variant'] !== undefined) 
                        result['scd_variant'] = data['scd_variant'];
                    return result
                }
            }
        ];

        const toZigbee = [
            {
                key: ['man_measure', 'factory_resets', 'scd_variant'],
                convertGet: async (entity, key, meta) => {
                    await entity.read('customSCD4x', [key]);
                },
            },
            {
                key: "cmd_factory_reset",
                convertSet: async (entity, key, value, meta) => {
                    await entity.command("customSCD4x", "factoryResetSCD4X", {}, {
                        disableDefaultResponse: true,
                    });
                },
            }
        ];

        const configure = [];

        configure.push(
            setupConfigureForReading("customSCD4x", ["man_measure", "factory_resets", "scd_variant"]),
            setupConfigureForReporting("customSCD4x", "man_measure", {
                config: {min: "1_SECOND", max: "MAX", change: 1},
                access: ea.STATE_GET,
            }),
            setupConfigureForReporting("customSCD4x", "factory_resets", {
                config: {min: "1_SECOND", max: "MAX", change: 1},
                access: ea.STATE_GET,
            }),
        );

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    extendedStatus: () => {
        const exposes = [
            e.numeric('status1', ea.STATE_GET).withLabel('Status1').withCategory('diagnostic'),
            e.numeric('status2', ea.STATE_GET).withLabel('Status2').withCategory('diagnostic'),
            e.numeric('status3', ea.STATE_GET).withLabel('Status3').withCategory('diagnostic'),
        ];

        const fromZigbee = [
            {
                cluster: 'customStatus',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    if (data['status1'] !== undefined) 
                        result['status1'] = data['status1'];
                    if (data['status2'] !== undefined) 
                        result['status2'] = data['status2'];
                    if (data['status3'] !== undefined) 
                        result['status3'] = data['status3'];
                    return result
                }
            }
        ];

        const toZigbee = [
            {
                key: ['status1', 'status2', 'status3'],
                convertGet: async (entity, key, meta) => {
                    await entity.read('customStatus', [key]);
                },
            }
        ];

        const configure = [];

        configure.push(
            setupConfigureForReading("customStatus", ["status1", "status2", "status3"]),
            setupConfigureForReporting("customStatus", "status1", {
                config: {min: "1_SECOND", max: "MAX", change: 1},
                access: ea.STATE_GET,
            }),
            setupConfigureForReporting("customStatus", "status2", {
                config: {min: "1_SECOND", max: "MAX", change: 1},
                access: ea.STATE_GET,
            }),
            setupConfigureForReporting("customStatus", "status3", {
                config: {min: "1_SECOND", max: "MAX", change: 1},
                access: ea.STATE_GET,
            }),
        );
        return {
            exposes,
            fromZigbee,
            toZigbee,
            configure,
            isModernExtend: true,
        };
    }
};

const definition = {
    zigbeeModel: ['CO2-NG'],
    model: 'CO2-54L15-NG',
    fingerprint: [{modelID: 'CO2-NG', applicationVersion: 1, priority: -1},],
    vendor: 'SFINAE',
    description: 'CO2 Sensor',
    extend: [
        deviceAddCustomCluster('customStatus', {
            ID: 0xfc80,
            attributes: {
                status1: {ID: 0x0000, type: Zcl.DataType.INT16},
                status2: {ID: 0x0001, type: Zcl.DataType.INT16},
                status3: {ID: 0x0002, type: Zcl.DataType.INT16},
            },
            commands: {},
            commandsResponse: {}
        }),
        deviceAddCustomCluster('customSCD4x', {
            ID: 0xfc01,
            attributes: {
                man_measure: {ID: 0x0000, type: Zcl.DataType.UINT8},
                factory_resets: {ID: 0x0001, type: Zcl.DataType.UINT8},
                scd_variant: {ID: 0x0002, type: Zcl.DataType.UINT16},
            },
            commands: {
                factoryResetSCD4X: {
                    ID: 0x00,
                    parameters: [],
                },
            },
            commandsResponse: {}
        }),
        battery({
            voltage: true, 
            voltageReporting: true
        }),
        co2(),
        temperature(),
        humidity(),
        orlangurSCD4XExtended.scd4x(),
        orlangurSCD4XExtended.extendedStatus()
    ],
    configure: async (device, coordinatorEndpoint) => {
        //dummy
    },

};

module.exports = definition;
