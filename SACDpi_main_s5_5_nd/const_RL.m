
%Constructs RL components:
%RL Algorithm: SAC

ObsInfo = rlNumericSpec([81 1]);
ObsInfo.Name = "Network states + demand states";
ObsInfo.Description = 'network states x, 1 by 75 vector; demand values, 1 by 6 vector';

ActInfo = rlNumericSpec([2 1],"UpperLimit", ones(2,1), "LowerLimit", zeros(2,1));
ActInfo.Name = "Ramp metering rates";




%%%%%%%%%%%

statePath = [featureInputLayer(prod(ObsInfo.Dimension),...
                'Normalization','none','Name','state')
                fullyConnectedLayer(256,'Name', 'fc1_state')];
            
actionPath = [featureInputLayer(prod(ActInfo.Dimension), ...
    'Normalization','none','Name','action')
    fullyConnectedLayer(128,"Name",'fc_2_action')];


commonPath = [concatenationLayer(1,2,'Name','concat')
              reluLayer('Name','reLu')
              fullyConnectedLayer(256, ...
                'Name','StateValue')
                reluLayer('Name', 'relu_body')
                fullyConnectedLayer(128, 'Name','fc_body')
                reluLayer('Name','relu_body2')
                fullyConnectedLayer(1,'Name','output')];

%criticNetwork = layerGraph(statePath);
criticNetwork = dlnetwork;
criticNetwork = addLayers(criticNetwork, statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = addLayers(criticNetwork, commonPath);
criticNetwork = connectLayers(criticNetwork, 'fc1_state','concat/in1');
criticNetwork = connectLayers(criticNetwork, 'fc_2_action','concat/in2');
criticNetwork1 = initialize(criticNetwork);
criticNetwork2 = initialize(criticNetwork);
critic1 = rlQValueFunction(criticNetwork1,ObsInfo,ActInfo);
critic2 = rlQValueFunction(criticNetwork2,ObsInfo,ActInfo);


actorPath = [featureInputLayer(prod(ObsInfo.Dimension),...
                'Normalization','none','Name','actInLyr')
                fullyConnectedLayer(256,'Name', 'fc1_actor')
                reluLayer(Name="CommonOutLyr")];

meanPath = [fullyConnectedLayer(256,'Name','MeanInLyr')
     reluLayer('Name','MeanRelu1')
     fullyConnectedLayer(prod(ActInfo.Dimension),'Name','MeanOutLyr')     ];

stdPath = [fullyConnectedLayer(256,'Name','StdInLyr')
     reluLayer('Name','StdRelu1')
     fullyConnectedLayer(prod(ActInfo.Dimension),'Name','StdFC2')     
     softplusLayer(Name="StdOutLyr")];

actorNet = dlnetwork;
actorNet = addLayers(actorNet,actorPath);
actorNet = addLayers(actorNet,meanPath);
actorNet = addLayers(actorNet,stdPath);
actorNet = connectLayers(actorNet, 'CommonOutLyr','MeanInLyr/in');
actorNet = connectLayers(actorNet, 'CommonOutLyr','StdInLyr/in');
actorNet = initialize(actorNet);


actor  = rlContinuousGaussianActor(actorNet,ObsInfo,ActInfo, ...
    ActionMeanOutputNames='MeanOutLyr',ActionStandardDeviationOutputNames='StdOutLyr',ObservationInputNames="actInLyr");



%activate GPU training:
%critic.UseDevice = "gpu";
%actor.UseDevice = "gpu";

%consider parametrizing the learning parameters since they depend on
%MPC_param.M_RL and MPC_param.M
agent = rlSACAgent(actor,[critic1,critic2]);
agent.AgentOptions.MiniBatchSize=512; %10; %MPC horizon
agent.AgentOptions.ExperienceBufferLength=2e5; %100; %Single scenario length, 10*mini batch size
agent.AgentOptions.TargetSmoothFactor=1e-2;
agent.AgentOptions.TargetUpdateFrequency=10; %5; %Previously it was 10
agent.AgentOptions.DiscountFactor=0.99;
agent.AgentOptions.NumStepsToLookAhead=10; %5; %MPC control step
%agent.AgentOptions.LearningFrequency=10; %MPC control step, keeps the MPC actions optimal by ensuring the agent is stationary during an episode
%agent.AgentOptions.NumWarmStartSteps=512; %Selected as the mini batch size, which is the possible minimum value
agent.AgentOptions.ActorOptimizerOptions.LearnRate=0.001;
agent.AgentOptions.ActorOptimizerOptions.GradientThreshold=1;

for ci=1:2
    agent.AgentOptions.CriticOptimizerOptions(ci).LearnRate=0.001;
    agent.AgentOptions.CriticOptimizerOptions(ci).GradientThreshold=1;
end


StepHandle = @(Action,Info) rlStepFunc(Action, Info, agent);
ResetHandle = @() rlResFunc;

env = rlFunctionEnv(ObsInfo,ActInfo,StepHandle,ResetHandle);

%activate parallel pools using GPUs
%availableGPUs = gpuDeviceCount("available");
%parpool("Processes",availableGPUs);


opt = rlTrainingOptions('MaxEpisodes',3500,... %previously it was 3000 episodes
                                          'MaxStepsPerEpisode',150,'UseParallel',true,... %previously maxsteps was 150 for 900 sim steps
                                          'SaveAgentCriteria','EpisodeFrequency',...
                                          'SaveAgentValue',50, 'SaveAgentDirectory', pwd + "\run\Agents", 'Plots',"none");
%                                          'MaxStepsPerEpisode',150,'UseParallel',false,'SaveAgentCriteria','AverageReward','SaveAgentValue',-1300);%,...
%                                         'StopTrainingCriteria','AverageReward',...
%                                         'StopTrainingValue',-1200,...
%                                         'UseParallel',false,...
%                                         'SaveAgentCriteria','AverageReward',...
%                                         'SaveAgentValue',-1200);

opt.ParallelizationOptions.Mode = "async";

%%%%%%%%%%%

trainResults = train(agent,env,opt);

agent_doc_name = 'agent_' + string(datetime('now'), 'yyyy-MM-dd hh_mm_ss') + '.mat';
save(agent_doc_name, "agent","trainResults")

%terminate parallel pools
delete(gcp("nocreate"));