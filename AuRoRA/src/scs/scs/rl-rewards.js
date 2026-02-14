// js/rl-rewards.js - RL reward system and feedback handling
const Rewards = {
    elements: {},
    state: {
        latestResponseId: null,
        positiveRewardCount: 0,
        negativeRewardCount: 0
    },

    init() {
        this.elements = {
            negativeRobotBtn: document.getElementById('negativeRobotBtn'),
            negCounter: document.getElementById('negCounter'),
            rewardFeedback: document.getElementById('rewardFeedback'),
            speechBubble: document.getElementById('speechBubble')
        };
    },

    generateResponseId() {
        return 'resp_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
    },

    showRewardFeedback(type, intensity) {
        const feedback = this.elements.rewardFeedback;
        feedback.className = 'reward-feedback ' + (type > 0 ? 'positive' : 'negative');
        feedback.textContent = type > 0 ? '👍 +' + intensity : '👎 ' + intensity;
        feedback.classList.add('show');
        
        setTimeout(() => {
            feedback.classList.remove('show');
        }, 1000);
    },

    sendRLReward(rewardType, intensity, responseId) {
        const isROSConnected = window.Chat && window.Chat.state.isROSConnected;
        const rlRewardTopic = window.rlRewardTopic;
        
        if (!isROSConnected || !rlRewardTopic) {
            console.log('RL Reward (offline):', { type: rewardType, intensity, responseId });
            return;
        }
        
        const rewardData = {
            response_id: responseId,
            reward_type: rewardType,
            intensity: intensity,
            timestamp: Date.now()
        };
        
        rlRewardTopic.publish(new ROSLIB.Message({
            data: JSON.stringify(rewardData)
        }));
        
        console.log('RL Reward sent:', rewardData);
    },

    handleNegativeReward() {
        if (!this.state.latestResponseId) {
            this.elements.speechBubble.textContent = 'No response to rate yet! 💭';
            this.elements.speechBubble.classList.add('show');
            setTimeout(() => this.elements.speechBubble.classList.remove('show'), 2000);
            return;
        }
        
        this.state.negativeRewardCount++;
        const intensity = Math.min(this.state.negativeRewardCount, 5);
        
        // Visual feedback
        this.showRewardFeedback(-1, intensity);
        
        // Update counter badge
        this.elements.negCounter.textContent = this.state.negativeRewardCount;
        this.elements.negCounter.classList.add('show');
        
        // Robot shake animation
        this.elements.negativeRobotBtn.classList.add('shaking');
        setTimeout(() => this.elements.negativeRobotBtn.classList.remove('shaking'), 500);
        
        // Send to ROS
        this.sendRLReward('negative', intensity, this.state.latestResponseId);
        
        // Grace reaction
        Robot.setEmotion('sad', '😢');
        this.elements.speechBubble.textContent = ['😢', '😭', '💔', '😞'][Math.min(intensity - 1, 3)] + ' I\'ll try better!';
        this.elements.speechBubble.classList.add('show');
        
        setTimeout(() => {
            this.elements.speechBubble.classList.remove('show');
            if (!Robot.state.isBeingPetted) Robot.setEmotion('happy');
        }, 2000);
        
        // Reset after delay for next response
        setTimeout(() => {
            this.state.negativeRewardCount = 0;
            this.elements.negCounter.classList.remove('show');
        }, 5000);
    }
};