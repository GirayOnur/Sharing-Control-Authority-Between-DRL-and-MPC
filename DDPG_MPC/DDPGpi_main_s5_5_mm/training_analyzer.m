%Plots the training curve of a saved agent: average reward per episode and
%the critic's estimate of the return at the start of each episode (Q0).
%Point it at the agent .mat file you want to look at.

agent_mat = load("agent_2025-08-21 08_39_40.mat");

trainResults = agent_mat.trainResults;

episodeNum = size(trainResults.EpisodeIndex,1);

episodeAx = linspace(1,episodeNum,episodeNum);

plot(episodeAx,trainResults.AverageReward,'b')
hold on
plot(episodeAx,trainResults.EpisodeQ0,'k')



% agent_mat = load("Agent3300.mat");
% 
% trainResults = agent_mat.savedAgentResult;
% 
% episodeNum = size(trainResults.EpisodeIndex,1);
% 
% episodeAx = linspace(1,episodeNum,episodeNum);
% 
% plot(episodeAx,trainResults.AverageReward,'b')
% hold on
% plot(episodeAx,trainResults.EpisodeQ0,'k')
% 

