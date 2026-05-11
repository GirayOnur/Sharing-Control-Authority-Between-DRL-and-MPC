
agent_mat = load("agent_2025-07-11 11_53_38.mat");

trainResults = agent_mat.trainResults;

episodeNum = size(trainResults.EpisodeIndex,1);

episodeAx = linspace(1,episodeNum,episodeNum);

plot(episodeAx,trainResults.AverageReward,'b')
hold on
plot(episodeAx,trainResults.EpisodeQ0,'k')



% trainResults = agent_mat.savedAgentResult;
% 
% episodeNum = size(trainResults.EpisodeIndex,1);
% 
% episodeAx = linspace(1,episodeNum,episodeNum);
% 
% plot(episodeAx,trainResults.AverageReward,'b')
% hold on
% plot(episodeAx,trainResults.EpisodeQ0,'k')


