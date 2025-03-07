local data = require('data.min')
local plain_text = require('plain_text.min')

-- Phone to Frame flags
TEXT_FLAG = 0x0a

-- register the message parsers so they are automatically called when matching data comes in
data.parsers[TEXT_FLAG] = plain_text.parse_plain_text

-- draw the specified text on the display
function print_text(text)
    local i = 0
    for line in text:gmatch('([^\n]*)\n?') do
        if line ~= "" then
            frame.display.text(line, 1, i * 60 + 1)
            i = i + 1
        end
    end
end

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

					if data.app_data[TEXT_FLAG] ~= nil and data.app_data[TEXT_FLAG].string ~= nil then
						local text = data.app_data[TEXT_FLAG]
						print_text(text.string)
						frame.display.show()

						-- clear the object and run the garbage collector right away
						data.app_data[TEXT_FLAG] = nil
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