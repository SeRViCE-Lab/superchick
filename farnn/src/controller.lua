--[[ 	
		This code publishes the controller messages 
		to myrio which in turn controls the valve circuit boards

		Author: Olalekan Ogunmolu
		Date: Nov 9, 2016
		]]

require 'torch'
require 'nn'
require 'rnn'
require 'nngraph'

local ros = require 'ros'

ros.init('service_client_demo')

spinner = ros.AsyncSpinner()
spinner:start()

local nh = ros.NodeHandle()
local clientA = nh:serviceClient('/error_srv', 'nn_controller/amfcError', true)
local ok = clientA:exists()
local timeout = ros.Duration(5)
local ok = clientA:waitForExistence(timeout)
-- print('waitForExistence() returned: ' .. tostring(ok))
print('Calling service: ' .. clientA:getService())
local sc = ros.ServiceClient('/error_srv', 'nn_controller/amfcError', true)

local function net_controller()
	local net_controller = nn.Sequential()
	net_controller:add(nn.Linear(1, 3))
	net_controller:add(nn.Sigmoid())
	net_controller:add(nn.Linear(3, 1))
	local cost = nn.MSECriterion()
	return cost, net_controller
end


local params = clientA:call()
local am, km, ref, target = 	params.am, params.km, params.ref, params.y

local function get_target()
	local clientA = nh:serviceClient('/error_srv', 'nn_controller/amfcError', true)
	local ok = clientA:exists()
	local target, err, input

	if ok then 
		local param = clientA:call()
		target, err = param.y, param.error
	end
	local y = torch.DoubleTensor(1):fill(target); 
	local e = torch.DoubleTensor(1):fill(err);
	local input = target
	return input
end

cost, neunet = net_controller()
print('neunet: ', neunet)

local iter = 1
local floatSpec = ros.MsgSpec('std_msgs/Float64')

local function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

publisher = nh:advertise("/controller", floatSpec, 100, false, connect_cb, disconnect_cb)
control   = ros.Message(floatSpec)

while ros.ok() do
	local target = get_target()	
	local pitch = torch.DoubleTensor(1):fill(params.y)

	neunet:zeroGradParameters()
	neunet:forget()
	iter = iter + 1

    if iter % 10  == 0 then collectgarbage() end
	local netout = neunet:forward(pitch)
	local loss 	 = cost:forward(netout, pitch)
	local grad   = cost:backward(netout, pitch)
	local gradIn = neunet:backward(pitch, grad)

	local u = -am * target + km * ref + netout[1]
	ros.INFO("actual: %d, pred: %4.4f loss: %d control: %3.4f", params.y, netout[1], loss, u)

	if publisher:getNumSubscribers() == 0 then
	  print('waiting for subscriber')
	else
	  control.data = u
	  publisher:publish(control)
	end

	ros.spinOnce(true)
	sys.sleep(0.2)
end

ros.shutdown()


