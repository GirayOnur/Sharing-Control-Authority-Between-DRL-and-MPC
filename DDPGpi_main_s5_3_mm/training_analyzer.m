
agent_mat = load("agent_2025-08-21 07_21_26.mat");

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

