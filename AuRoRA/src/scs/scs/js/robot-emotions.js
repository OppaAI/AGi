// Robot elements
        const robot = document.getElementById('robot');
        const petZone = document.getElementById('petZone');
        const robotHead = document.getElementById('robotHead');
        const leftEye = document.getElementById('leftEye');
        const rightEye = document.getElementById('rightEye');
        const mouth = document.getElementById('mouth');
        const speechBubble = document.getElementById('speechBubble');
        const speakingWaves = document.getElementById('speakingWaves');
        const emotionIndicator = document.getElementById('emotionIndicator');
        const petCounter = document.getElementById('petCounter');
        const chatContainer = document.getElementById('chatContainer');
        const statusDot = document.getElementById('statusDot');
        const statusText = document.getElementById('statusText');
        const userInput = document.getElementById('userInput');
        const sendBtn = document.getElementById('sendBtn');
        const plusMenu = document.getElementById('plusMenu');
        const globeBtn = document.getElementById('globeBtn');
        const negativeRobotBtn = document.getElementById('negativeRobotBtn');
        const negCounter = document.getElementById('negCounter');
        const rewardFeedback = document.getElementById('rewardFeedback');
        
        // Image elements
        const imagePreviewArea = document.getElementById('imagePreviewArea');
        const previewImg = document.getElementById('previewImg');
        const imagePrompt = document.getElementById('imagePrompt');
        let currentImage = null;
        let imageInputTopic = null;
        
        // State variables
        let isSpeaking = false;
        let isStreaming = false;
        let idleInterval;
        let blinkInterval;
        let currentStreamMessage = null;
        let streamBuffer = '';
        let currentEmotion = 'happy';
        let petCount = 0;
        let isBeingPetted = false;
        let petTimeout;
        let ros = null;
        let isROSConnected = false;
        let webSearchEnabled = false;
        
        // RL Reward System
        let latestResponseId = null;
        let positiveRewardCount = 0;
        let negativeRewardCount = 0;
        let rlRewardTopic = null;
        
        const petResponses = [
            "💖: Aww, that feels nice!",
            "🥰: I love it when you pet me!",
            "💕: You're so sweet!",
            "✨: *happy robot noises*",
            "💗: More please!",
            "🤗: That tickles!",
            "💝: You're the best!",
            "🥺: *blushes*"
        ];
        
        const emotionMap = {
            '😊': 'happy', '😄': 'happy', '😃': 'happy', '😁': 'happy', '😆': 'happy',
            '🥰': 'loving', '😍': 'loving', '🥳': 'happy', '✨': 'happy', '💖': 'loving',
            '💕': 'loving', '💗': 'loving', '💝': 'loving', '💘': 'loving',
            '😢': 'sad', '😭': 'sad', '😔': 'sad', '😟': 'sad', '😞': 'sad',
            '😥': 'sad', '💔': 'sad', '😿': 'sad',
            '😠': 'angry', '😡': 'angry', '😤': 'angry', '👿': 'angry', '💢': 'angry',
            '😮': 'surprised', '😲': 'surprised', '😯': 'surprised', '😱': 'surprised',
            '😳': 'surprised', '🤯': 'surprised',
            '🤔': 'thinking', '😕': 'thinking', '🧐': 'thinking', '🤨': 'thinking',
            '❓': 'thinking', '❔': 'thinking',
            '❤️': 'loving', '💓': 'loving', '💞': 'loving', '💟': 'loving',
            'default': 'happy'
        };
        
        // Plus menu toggle
        function togglePlusMenu() {
            plusMenu.classList.toggle('show');
        }
        
        // Globe toggle for web search
        function toggleWebSearch() {
            webSearchEnabled = !webSearchEnabled;
            
            if (webSearchEnabled) {
                globeBtn.classList.add('active');
                speechBubble.textContent = '🌐 Web search enabled!';
                setEmotion('thinking', '🌐');
            } else {
                globeBtn.classList.remove('active');
                speechBubble.textContent = 'Web search disabled';
                setEmotion('happy', '💕');
            }
            
            speechBubble.classList.add('show');
            setTimeout(() => {
                speechBubble.classList.remove('show');
                if (!isBeingPetted) setEmotion('happy');
            }, 2000);
        }
        
        // RL Reward Functions
        function generateResponseId() {
            return 'resp_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
        }
        
        function showRewardFeedback(type, intensity) {
            rewardFeedback.className = 'reward-feedback ' + (type > 0 ? 'positive' : 'negative');
            rewardFeedback.textContent = type > 0 ? '👍 +' + intensity : '👎 ' + intensity;
            rewardFeedback.classList.add('show');
            
            setTimeout(() => {
                rewardFeedback.classList.remove('show');
            }, 1000);
        }
        
        function sendRLReward(rewardType, intensity, responseId) {
            if (!isROSConnected || !rlRewardTopic) {
                console.log('RL Reward (offline):', { type: rewardType, intensity, responseId });
                return;
            }
            
            const rewardData = {
                response_id: responseId,
                reward_type: rewardType, // 'positive' or 'negative'
                intensity: intensity,    // 1, 2, 3... based on click count
                timestamp: Date.now()
            };
            
            rlRewardTopic.publish(new ROSLIB.Message({
                data: JSON.stringify(rewardData)
            }));
            
            console.log('RL Reward sent:', rewardData);
        }
        
        function handleNegativeReward() {
            if (!latestResponseId) {
                speechBubble.textContent = 'No response to rate yet! 💭';
                speechBubble.classList.add('show');
                setTimeout(() => speechBubble.classList.remove('show'), 2000);
                return;
            }
            
            negativeRewardCount++;
            const intensity = Math.min(negativeRewardCount, 5); // Cap at 5
            
            // Visual feedback
            showRewardFeedback(-1, intensity);
            
            // Update counter badge
            negCounter.textContent = negativeRewardCount;
            negCounter.classList.add('show');
            
            // Robot shake animation
            negativeRobotBtn.classList.add('shaking');
            setTimeout(() => negativeRobotBtn.classList.remove('shaking'), 500);
            
            // Send to ROS
            sendRLReward('negative', intensity, latestResponseId);
            
            // Grace reaction
            setEmotion('sad', '😢');
            speechBubble.textContent = ['😢', '😭', '💔', '😞'][Math.min(intensity - 1, 3)] + ' I\'ll try better!';
            speechBubble.classList.add('show');
            
            setTimeout(() => {
                speechBubble.classList.remove('show');
                if (!isBeingPetted) setEmotion('happy');
            }, 2000);
            
            // Reset after delay for next response
            setTimeout(() => {
                negativeRewardCount = 0;
                negCounter.classList.remove('show');
            }, 5000);
        }
        
        // Close menu when clicking outside
        document.addEventListener('click', (e) => {
            if (!e.target.closest('.plus-btn') && !e.target.closest('.plus-menu')) {
                plusMenu.classList.remove('show');
            }
        });
        
        // Image handling
        function handleImageSelect(input) {
            const file = input.files[0];
            if (!file) return;
            
            if (!file.type.startsWith('image/')) {
                alert('Please select an image file');
                return;
            }
            
            if (file.size > 5 * 1024 * 1024) {
                alert('Image too large! Please select an image under 5MB');
                return;
            }
            
            const reader = new FileReader();
            reader.onload = function(event) {
                currentImage = event.target.result;
                previewImg.src = currentImage;
                imagePreviewArea.classList.add('show');
                imagePrompt.focus();
                plusMenu.classList.remove('show');
                
                // Auto-focus the image prompt input
                setTimeout(() => imagePrompt.focus(), 100);
            };
            
            reader.onerror = function() {
                alert('Error reading image file');
            };
            
            reader.readAsDataURL(file);
        }
        
        function removeImage() {
            currentImage = null;
            imagePreviewArea.classList.remove('show');
            document.getElementById('imageInput').value = '';
            document.getElementById('cameraInput').value = '';
            imagePrompt.value = '';
        }
        
        