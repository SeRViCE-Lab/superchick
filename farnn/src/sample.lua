require 'torch'
require 'nn'
require 'rnn'
require 'nngraph'

local cmd = torch.CmdLine()
local ros = require 'ros'

--Gpu settings
cmd:option('-gpu', 0, 'which gpu to use. -1 = use CPU; >=0 use gpu')
cmd:option('-checkpoint', 'sr-net/softRobot_lstm-net.t7', 'load the trained network e.g. <lstm-net.t7| rnn-net.t7|mlp-net.t7>')
cmd:option('-backend', 'cudnn', 'nn|cudnn')
cmd:option('-ros', 'use ros?')
cmd:option('-verbose', false)

local opt = cmd:parse(arg)
torch.setnumthreads(8)

ros.init('advertise_neunet')
nh = ros.NodeHandle()
string_spec = ros.MsgSpec('std_msgs/String')


local use_cuda = false

local msg
if opt.gpu >= 0  and opt.backend == 'cudnn' then
	require 'cunn'
	require 'cutorch'
	require 'cudnn'
	use_cuda = true
	cutorch.setDevice(opt.gpu + 1)
	msg = string.format('Code deployed on GPU %d with cudnn backend', opt.gpu+1)
else
  msg = 'Running in CPU mode'
  require 'nn'
end
if opt.verbose then print(msg) end

local checkpoint, model

if use_cuda then
	model = torch.load(opt.checkpoint)
	model:cuda()
else
	model = torch.load(opt.checkpoint)
	model:double()
end

model:evaluate()

netmods = model.modules;

weights,biases = {}, {};
netparams, netparams2 = {}, {}
local broadcast_weights = {}


publisher = nh:advertise("/saved_net", string_spec, 10, false, connect_cb, disconnect_cb)
msg 	  = ros.Message(string_spec)

while(ros.ok())	do
	if #netmods == 1 then   		--recurrent modules
		local modules 	= netmods[1].recurrentModule.modules
		local length 	= #modules
		for i = 1, length do
			netparams[i] 	= {['weight']=modules[i].weight, ['bias']=modules[i].bias}
		end

		-- find indices in netparams that contain weights
		for k, v in pairs(netparams) do 
			if netparams[k].weight then 
			   broadcast_weights = netparams[k].weight
			end
		end


		if opt.verbose then 
			print('\nbroadcast_weights\n:'); 
			print(broadcast_weights)
		end


	elseif #netmods > 1 then   --mlp modules
		for i = 1, #netmods do		
			netparams[i] 	= {['weight']=netmods[i].weight, ['bias']=netmods[i].bias}
		end

		-- find indices in netparams that contain weights
		for k, v in pairs(netparams) do 
			if netparams[k].weight then 
			   broadcast_weights = netparams[k].weight
			end
		end

		if opt.verbose then 
			print('\nnetparams\n:'); 
			print(netparams)
		end
	end

 	if publisher:getNumSubscribers() == 0 then
    	print('waiting for subscriber')
  	else
    	msg.data = tostring(broadcast_weights)
    	publisher:publish(msg)
  	end
  	sys.sleep(1)
  	ros.spinOnce()
end

ros.shutdown()

