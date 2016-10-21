--[[ This file measures how generalizable the network model is.
      Author: Olalekan Ogunmolu. 
      Date:   Oct 20, 2016
      Lab Affiliation: Gans' Lab
      ]]
require 'torch'
require 'data.dataparser'
require 'rnn'

local optnet = require 'optnet'
torch.setdefaulttensortype('torch.FloatTensor')

--options and general settings -----------------------------------------------------
opt = {
  batchSize = 50,
  data = 'softRobot',
  gpu = -1,
  noutputs = 1,
  display = 1,
  verbose = true,
  maxIter = 10000,
  hiddenSize = {2, 5, 10},
  checkpoint = 'network/softRobot_lstm-net.t7'
}

for k,v in pairs(opt) do opt[k] = tonumber(os.getenv(k)) or os.getenv(k) or opt[k] end
print(opt)

if opt.display==0 then opt.display = false end

local function init()

  --GPU Settings -----------------------------------------------------------------------
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
  if opt.verbose == 1 then print(msg) end
  -------------------------------------------------------------------------------

  ---Net Loader---------------------------------------------------------------------------
  local model
  model = torch.load(opt.checkpoint)

  assert(model ~='', 'must provide a trained model')

  if use_cuda then
    model:cuda()
  else
    model:double()
  end

  netmods = model.modules;
  if opt.verbose then print('netmods: \n', netmods) end
  --------------------------------------------------------------------------------
  return model
end


--test function-----------------------------------------------------------------
local function test(opt, model)
 -- local vars
 local splitData = {}; 
 splitData = split_data(opt)
 local time = sys.clock()
 local testHeight = splitData.test_input[1]:size(1)
 -- averaged param use?
 if average then
    cachedparams = parameters:clone()
    parameters:copy(average)
 end
 -- test over given dataset
 print('<trainer> on testing Set:')

  local preds;
  local avg = 0; local predF, normedPreds = {}, {}
  local iter = 0; local for_limit; 

  -- create mini batch        
  local inputs, targets = {}, {}      

  for t = 1, math.min(opt.maxIter, testHeight) do
    _, __, inputs, targets = get_datapair(opt, t)
    if opt.noutputs  == 1 then 
      --concat tensors 1 and 4 (in and pitch along 2nd dimension)
      inputs = torch.cat({inputs[1], inputs[4]}, 2) 
      -- target would be expected to be a tensor rather than a table since we are using sequencer
      targets = targets[3]
    else
      inputs = torch.cat({inputs[1], inputs[2], inputs[3], inputs[4], inputs[5], inputs[6], inputs[7]}, 2)
      targets = torch.cat({targets[1], targets[2], targets[3], targets[4], targets[5], targets[6]}, 2)
    end
    -- print('inputs: ', inputs)
    -- test samples
    local preds = model:forward(inputs)
  

    for_limit = preds:size(2)

    -- if for_limit == 1 then      
    predF = preds:float()
    normedPreds = torch.norm(predF)

    -- timing
    time = sys.clock() - time
    time = time / height

    if  (iter*opt.batchSize >= math.min(opt.maxIter, testHeight))  then 
      print("<trainer> time to test 1 sample = " .. (time*1000) .. 'ms')  
      if not (opt.data=='glassfurnace') then print("avg. prediction errors on test data", normedPreds) 
      else print("avg. prediction errors on test data", avg/normedPreds) end
    end 
    iter = iter + 1
  end  
end

local function main()
  local model = init()
  test(opt, model)
end

main()