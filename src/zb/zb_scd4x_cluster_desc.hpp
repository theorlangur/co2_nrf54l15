#ifndef ZB_SCD4X_CLUSTER_DESC_HPP_
#define ZB_SCD4X_CLUSTER_DESC_HPP_

#include <nrfzbcpp/zb_main.hpp>

namespace zb
{
    static constexpr uint16_t kZB_ZCL_CLUSTER_ID_SCD4X = 0xfc01;

    struct zb_zcl_scd4x_t
    {
        uint8_t manual_measurements = 0;
        uint8_t factory_resets = 0;
        uint16_t sensor_variant = 0;
        cmd_in_t<0> cmd_on_factory_reset;
    };

    template<> struct zcl_description_t<zb_zcl_scd4x_t> {
        static constexpr auto get()
        {
            using T = zb_zcl_scd4x_t;
            return cluster_struct_desc_t<
                cluster_info_t{.id = kZB_ZCL_CLUSTER_ID_SCD4X},
                attributes_t<
                    attribute_t{.m = &T::manual_measurements,  .id = 0x0000, .a=Access::RP}
                    ,attribute_t{.m = &T::factory_resets,      .id = 0x0001, .a=Access::RP}
                    ,attribute_t{.m = &T::sensor_variant,      .id = 0x0002, .a=Access::Read}
                >{},
                commands_t<
                    &T::cmd_on_factory_reset
                >{}
            >{};
        }
    };
}
#endif
