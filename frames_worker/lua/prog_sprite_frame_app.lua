local data = require('data.min')
local image_sprite_block = require('image_sprite_block.min')

-- Phone to Frame flags
IMAGE_SPRITE_BLOCK = 0x20

-- register the message parsers so they are automatically called when matching data comes in
data.parsers[IMAGE_SPRITE_BLOCK] = image_sprite_block.parse_image_sprite_block


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

					if (data.app_data[IMAGE_SPRITE_BLOCK] ~= nil) then
						-- show the image sprite block
						local isb = data.app_data[IMAGE_SPRITE_BLOCK]

						-- it can be that we haven't got any sprites yet, so only proceed if we have a sprite
						if isb.current_sprite_index > 0 then
							-- either we have all the sprites, or we want to do progressive/incremental rendering
							if isb.progressive_render or (isb.active_sprites == isb.total_sprites) then

								for index = 1, isb.active_sprites do
										local spr = isb.sprites[index]
										local y_offset = isb.sprite_line_height * (index - 1)

										-- set the palette the first time, all the sprites should have the same palette
										if index == 1 then
												image_sprite_block.set_palette(spr.num_colors, spr.palette_data)
										end

										frame.display.bitmap(1, y_offset + 1, spr.width, 2^spr.bpp, 0, spr.pixel_data)
								end

								frame.display.show()
							end
						end
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