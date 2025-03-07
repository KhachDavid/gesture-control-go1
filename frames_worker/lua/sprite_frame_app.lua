local data = require('data.min')
local sprite = require('sprite.min')

-- Phone to Frame flags
USER_SPRITE = 0x20

-- register the message parsers so they are automatically called when matching data comes in
data.parsers[USER_SPRITE] = sprite.parse_sprite


-- Main app loop
function app_loop()
	frame.display.text('Frame App Started', 1, 1)
	frame.display.show()

	while true do
        rc, err = pcall(
            function()
				-- process any raw data items, if ready
				local items_ready = data.process_raw_items()

				-- one or more full messages received
				if items_ready > 0 then

					if data.app_data[USER_SPRITE] ~= nil then
						local spr = data.app_data[USER_SPRITE]

						-- set the palette in case it's different to the standard palette
						sprite.set_palette(spr.num_colors, spr.palette_data)

						-- show the sprite
						frame.display.bitmap(1, 1, spr.width, 2^spr.bpp, 0, spr.pixel_data)
						frame.display.show()

						-- clear the object and run the garbage collector right away
						data.app_data[USER_SPRITE] = nil
						collectgarbage('collect')
					end

				end

				-- can't sleep for long, might be lots of incoming bluetooth data to process
				frame.sleep(0.001)
			end
		)
		-- Catch an error (including the break signal) here
		if rc == false then
			-- send the error back on the stdout stream and clear the display
			print(err)
			frame.display.text(' ', 1, 1)
			frame.display.show()
			break
		end
	end
end

-- run the main app loop
app_loop()