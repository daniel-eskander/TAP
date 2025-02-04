-- Copyright lowRISC Contributors.
-- SPDX-License-Identifier: Apache-2.0

set_project("Sonata Examples")
sdkdir = "../cheriot-rtos/sdk"
includes(sdkdir)
set_toolchains("cheriot-clang")

includes(path.join(sdkdir, "lib"))
includes("../libraries")
includes("../common.lua")

add_includedirs("../third_party/sonata-system/sw/cheri/tests")

option("board")
    set_default("sonata-prerelease")

includes("all", "snake", "automotive")

-- A simple demo using only devices on the Sonata board
firmware("sonata_simple_demo")
    add_deps("freestanding", "led_walk_raw", "echo")
    on_load(function(target)
        target:values_set("board", "$(board)")
        target:values_set("threads", {
            {
                compartment = "led_walk_raw",
                priority = 2,
                entry_point = "start_walking",
                stack_size = 0x200,
                trusted_stack_frames = 1
            },
            {
                compartment = "echo",
                priority = 1,
                entry_point = "entry_point",
                stack_size = 0x200,
                trusted_stack_frames = 1
            },
            -- {
            --     compartment = "lcd_test",
            --     priority = 2,
            --     entry_point = "lcd_test",
            --     stack_size = 0x1000,
            --     trusted_stack_frames = 1
            -- }
        }, {expand = false})
    end)
    after_link(convert_to_uf2)

-- A demo that expects additional devices such as I2C devices
firmware("sonata_demo_everything")
    add_deps("freestanding", "led_walk_raw", "echo",  "i2c_example")
    on_load(function(target)
        target:values_set("board", "$(board)")
        target:values_set("threads", {
            {
                compartment = "led_walk_raw",
                priority = 2,
                entry_point = "start_walking",
                stack_size = 0x200,
                trusted_stack_frames = 1
            },
            {
                compartment = "echo",
                priority = 1,
                entry_point = "entry_point",
                stack_size = 0x200,
                trusted_stack_frames = 1
            },
            -- {
            --     compartment = "lcd_test",
            --     priority = 2,
            --     entry_point = "lcd_test",
            --     stack_size = 0x1000,
            --     trusted_stack_frames = 1
            -- },
            {
                compartment = "i2c_example",
                priority = 2,
                entry_point = "run",
                stack_size = 0x300,
                trusted_stack_frames = 1
            }
        }, {expand = false})
    end)
    after_link(convert_to_uf2)

-- Demo that does proximity test as well as LCD screen, etc for demos.
firmware("sonata_proximity_demo")
    add_deps("freestanding", "led_walk_raw", "echo", "proximity_sensor_example")
    on_load(function(target)
        target:values_set("board", "$(board)")
        target:values_set("threads", {
            {
                compartment = "led_walk_raw",
                priority = 2,
                entry_point = "start_walking",
                stack_size = 0x200,
                trusted_stack_frames = 1
            },
            {
                compartment = "echo",
                priority = 1,
                entry_point = "entry_point",
                stack_size = 0x200,
                trusted_stack_frames = 1
            },
            -- {
            --     compartment = "lcd_test",
            --     priority = 2,
            --     entry_point = "lcd_test",
            --     stack_size = 0x1000,
            --     trusted_stack_frames = 1
            -- },
            {
                compartment = "proximity_sensor_example",
                priority = 2,
                entry_point = "run",
                stack_size = 0x200,
                trusted_stack_frames = 1
            }
        }, {expand = false})
    end)
    after_link(convert_to_uf2)

firmware("proximity_test")
    add_deps("freestanding", "proximity_sensor_example")
    on_load(function(target)
        target:values_set("board", "$(board)")
        target:values_set("threads", {
            {
                compartment = "proximity_sensor_example",
                priority = 2,
                entry_point = "run",
                stack_size = 0x200,
                trusted_stack_frames = 1
            }
        }, {expand = false})
    end)
    after_link(convert_to_uf2)


-- Demo that does proximity test as well as LCD screen, etc for demos.
firmware("DANIEL_DEMO")
    add_deps("freestanding", "led_walk_raw", "MAIN")
    on_load(function(target)
        target:values_set("board", "$(board)")
        target:values_set("threads", {
            {
                compartment = "led_walk_raw",
                priority = 2,
                entry_point = "start_walking",
                stack_size = 0x200,
                trusted_stack_frames = 1
            },
            {
                compartment = "MAIN",
                priority = 1,
                entry_point = "UART2_entry",
                stack_size = 0x1500,
                trusted_stack_frames = 1
            },
        }, {expand = false})
    end)
    after_link(convert_to_uf2)
