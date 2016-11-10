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

--options and general settings -----------------------------------------------------
local cmd = torch.CmdLine()
cmd:option('-model', 'lstm', 'mlp or lstm')
cmd:option('-hiddenSize', {1,3,1}, 'hidden size')
cmd:option('-rho', 5, 'BPTT steps')
cmd:option('-seed', 123, 'manual seed')
opt = cmd:parse(arg or {})
torch.manualSeed(opt.seed)
torch.setnumthreads(8)

for k,v in pairs(opt) do opt[k] = tonumber(os.getenv(k)) or os.getenv(k) or opt[k] end
print(opt)

ros.init('lmfc_controller')

spinner = ros.AsyncSpinner()
spinner:start()

local nh = ros.NodeHandle()
local lmfcClient = nh:serviceClient('/error_srv', 'nn_controller/amfcError', true)
local ok = lmfcClient:exists()
local timeout = ros.Duration(5)
local ok = lmfcClient:waitForExistence(timeout)

print('Calling service: ' .. lmfcClient:getService())
local sc = ros.ServiceClient('/error_srv', 'nn_controller/amfcError', true)

local function net_controller()
	local net_controller = nn.Sequential()
	if opt.model == 'mlp' then 
		net_controller:add(nn.Linear(1, 3))
		net_controller:add(nn.Sigmoid())
		net_controller:add(nn.Linear(3, 1))
	elseif opt.model == 'lstm' then
		nn.FastLSTM.usenngraph = false -- faster
		local hidden = opt.hiddenSize
		local inputSize = hidden[1]
		local rnn
		for i, hiddenSize in ipairs(hidden) do 
			rnn = nn.FastLSTM(inputSize, hiddenSize, opt.rho)
			net_controller:add(rnn)
		    inputSize = hiddenSize
		end
		net_controller = nn.Sequencer(net_controller, 1)
	end
	local cost = nn.MSECriterion()
	return cost, net_controller
end


local params = lmfcClient:call()
local am, km, ref, target = 	params.am, params.km, params.ref, params.y

local function get_target()
	local lmfcClient = nh:serviceClient('/error_srv', 'nn_controller/amfcError', true)
	local ok = lmfcClient:exists()
	local target, err, input, param

	if ok then 
		param = lmfcClient:call()
		target, err = param.y, param.error
	end
	local y = torch.DoubleTensor(1):fill(target); 
	local e = torch.DoubleTensor(1):fill(err);	
	return param, y, e
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
	local params = get_target()	
	local pitch = torch.DoubleTensor(1):fill(params.y)
	local target = params.y
	neunet:zeroGradParameters()
	neunet:forget()
	iter = iter + 1

    if iter % 10  == 0 then collectgarbage() end

    local netout, loss, grad, gradIn, u
	if opt.model == 'lstm' then 
		pitch = {pitch}
		netout = neunet:forward(pitch)
		loss 	 = cost:forward(netout[1], pitch[1])
		grad   = cost:backward(netout[1], pitch[1])
		gradIn = neunet:backward(pitch, {grad})
	else
		netout = neunet:forward(pitch)
		-- print('netout: ', netout, 'pitch', pitch)
		loss 	 = cost:forward(netout, pitch)
		grad   = cost:backward(netout, pitch)
		gradIn = neunet:backward(pitch, grad)
	end

	local u = -am * target + km * ref + netout[1]
	-- print('am: ', am, 'target: ', target, 'km: ', km, 'ref: ', ref, 'netout: ', netout[1], 'u: ', u[1])
	if opt.model=='lstm' then 
		ros.INFO("actual: %4.2f, pred: %4.4f, loss: %4.4f, control: %4.4f", params.y, tonumber(netout[1][1]), loss, u[1])
	  	control.data = u[1]
	else
		ros.INFO("actual: %d, pred: %4.4f loss: %d control: %3.4f", params.y, netout[1], loss, u)
	  	control.data = u
	end

	if publisher:getNumSubscribers() == 0 then
	  print('waiting for subscriber')
	else
	  publisher:publish(control)
	end

	ros.spinOnce(true)
	sys.sleep(0.2)
end

ros.shutdown()


