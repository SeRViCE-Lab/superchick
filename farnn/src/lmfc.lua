require 'torch'
require 'nn'
require 'rnn'
require 'nngraph'

local ros = require 'ros'

ros.init('service_client_demo')

local nh = ros.NodeHandle()
local clientA = nh:serviceClient('/error_srv', 'nn_controller/amfcError')
-- we can check if the service exists
local ok = clientA:exists()
print('exists() returned: ' .. tostring(ok))
-- or wait for it to become available
local timeout = ros.Duration(5)
local ok = clientA:waitForExistence(timeout)
print('waitForExistence() returned: ' .. tostring(ok))
print('Calling service: ' .. clientA:getService())
-- call the service
local response = clientA:call()

ros.INFO('Response: %f', response.error)

ros.spinOnce(30)

ros.shutdown()


