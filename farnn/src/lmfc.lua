require 'torch'
require 'nn'
require 'rnn'
require 'nngraph'

local ros = require 'ros'

ros.init('service_client_demo')

local nh = ros.NodeHandle()
local clientA = nh:serviceClient('/error_srv', 'nn_controller/amfcError', true)
-- we can check if the service exists
local ok = clientA:exists()
print('exists() returned: ' .. tostring(ok))
-- or wait for it to become available
local timeout = ros.Duration(5)
local ok = clientA:waitForExistence(timeout)
print('waitForExistence() returned: ' .. tostring(ok))
print('Calling service: ' .. clientA:getService())
-- call the service

local sc = ros.ServiceClient('/error_srv', 'nn_controller/amfcError', true)
local isPersistent = sc:isPersistent()
ros.INFO("Persistence Status: ", isPersistent)


while ros.ok() do
	local response = clientA:call()
	ros.INFO('Response: %f', response.error)
	ros.spinOnce(true)
	sys.sleep(0.2)
end

ros.shutdown()


