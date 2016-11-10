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
cmd:option('-hiddenSize', {1,10,1}, 'hidden size')
cmd:option('-outputSize', 1, 'output size')
cmd:option('-rho', 5, 'BPTT steps')
cmd:option('-rnnlearningRate', 1e-1, 'learning rate')
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
		net_controller = nn.Sequencer(net_controller, opt.outputSize)
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

while ros.ok() and ok do
	local params = get_target()	
	local pitch = torch.DoubleTensor(1):fill(params.y)
	local target = params.y
	neunet:zeroGradParameters()
	neunet:forget()
	iter = iter + 1

    if iter % 10  == 0 then collectgarbage() end

    local netout, loss, grad, gradIn, u
    local w, B
    local net 	= {}
	if opt.model== 'lstm' then 
		pitch  	= {pitch}
		netout 	= neunet:forward(pitch)
		loss   	= cost:forward(netout[1], pitch[1])
		grad   	= cost:backward(netout[1], pitch[1])
		gradIn 	= neunet:backward(pitch, {grad})

		neunet:updateParameters(opt.rnnlearningRate)


		--aggregate the weights of the input and recuurent layers
		net.input_weights 	= neunet.modules[1].modules[1].gradInput
		net.prediction 		= neunet.modules[1].output
		net.recurrent_weights, net.recurrent_outputs = {}, {}

		for i = 1, #opt.hiddenSize do 
			table.insert(net.recurrent_weights, neunet.modules[1].recurrentModule.modules[i].gradInput)
			table.insert(net.recurrent_outputs, neunet.modules[1].recurrentModule.modules[i].outputs[1])
		end

		--W \in r^P is the weight vector of the input units
		w = net.input_weights
		B = net.recurrent_weights
		B = torch.cat(B[1], B[2], B[3])
	else
		netout = neunet:forward(pitch)
		loss 	 = cost:forward(netout, pitch)
		grad   = cost:backward(netout, pitch)
		gradIn = neunet:backward(pitch, grad)
	end

	local Nf = B*w
	-- print('Nf: ', Nf)
	local u = -am * target + km * ref + Nf
	if opt.model=='lstm' then 
		ros.INFO("actual: %4.2f, pred: %4.8f, ref: %3.4f, loss: %4.4f, control: %4.4f", params.y, 
			        net.prediction[1], params.ref, loss, u)
	  	control.data = u
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


		--[[ 
		neunet.modules[1].sharedClones[1].gradInput = Input gradients
		neunet.modules[1].sharedClones[1].output = predictions
		neunet.modules[1].outputs[1] = predictions
		neunet.modules[1].output = predictions

		neunet.modules[1].gradInput = input gradients
		neunet.modules[1].modules[1].gradInput = input gradients
		neunet.modules[1].modules[1].output = prediction
		neunet.modules[1].recurrentModule.output = prediction


		print('recurrent layer 1 input weights: ', neunet.modules[1].recurrentModule.modules[1].gradInput)
		print('recurrent layer 2 input weights: ', neunet.modules[1].recurrentModule.modules[2].gradInput)
		print('recurrent layer 3 input weights: ', neunet.modules[1].recurrentModule.modules[3].gradInput)
		print('recurrent layer 1 input outs: ', neunet.modules[1].recurrentModule.modules[1].outputs[1])
		print('recurrent layer 2 input outs: ', neunet.modules[1].recurrentModule.modules[2].outputs[1])
		print('recurrent layer 3 input outs: ', neunet.modules[1].recurrentModule.modules[3].outputs[1])
		]]