// ROS Initialization
        function initROS() {
            try {
                ros = new ROSLIB.Ros({ url: 'ws://aurora-usb:9090' });
                
                ros.on('connection', () => {
                    isROSConnected = true;
                    statusDot.classList.add('connected');
                    statusText.textContent = 'Connected! 💖';
                    speechBubble.textContent = 'Connected! 💕';
                    speechBubble.classList.add('show');
                    setTimeout(() => speechBubble.classList.remove('show'), 2000);
                    startIdleAnimations();
                });
                
                ros.on('error', () => {
                    statusText.textContent = 'Offline Mode';
                    speechBubble.textContent = 'Hello! 💕';
                    speechBubble.classList.add('show');
                    setTimeout(() => speechBubble.classList.remove('show'), 3000);
                    startIdleAnimations();
                });
                
                ros.on('close', () => {
                    isROSConnected = false;
                    statusDot.classList.remove('connected');
                    statusText.textContent = 'Offline Mode';
                    stopIdleAnimations();
                });
                
                // Text input topic
                const cnsTopic = new ROSLIB.Topic({
                    ros: ros,
                    name: '/cns/neural_input',
                    messageType: 'std_msgs/String'
                });
                
                // Image input topic
                imageInputTopic = new ROSLIB.Topic({
                    ros: ros,
                    name: '/cns/image_input',
                    messageType: 'std_msgs/String'
                });
                
                // RL Reward topic
                rlRewardTopic = new ROSLIB.Topic({
                    ros: ros,
                    name: '/cns/rl_reward',
                    messageType: 'std_msgs/String'
                });
                
                // Response topic
                const gceResponseTopic = new ROSLIB.Topic({
                    ros: ros,
                    name: '/gce/response',
                    messageType: 'std_msgs/String'
                });
                
                gceResponseTopic.subscribe((msg) => {
                    try {
                        const data = JSON.parse(msg.data);
                        if (data.type === 'start') {
                            detectAndSetEmotion(data.content);
                            startStreaming(data.content);
                        } else if (data.type === 'delta') {
                            appendToStream(data.content);
                        } else if (data.type === 'done') {
                            finishStreaming();
                            // Generate new response ID for RL tracking
                            latestResponseId = generateResponseId();
                            positiveRewardCount = 0;
                            negativeRewardCount = 0;
                            negCounter.classList.remove('show');
                            console.log('New response ID:', latestResponseId);
                        } else if (data.type === 'error') {
                            if (currentStreamMessage) finishStreaming();
                            setEmotion('sad');
                            createMessageElement('grace', data.content);
                        }
                    } catch (e) {
                        if (!isStreaming) {
                            detectAndSetEmotion(msg.data);
                            startStreaming(msg.data);
                        } else {
                            appendToStream(msg.data);
                        }
                    }
                });
                
                window.cnsTopic = cnsTopic;
                window.imageInputTopic = imageInputTopic;
                window.rlRewardTopic = rlRewardTopic;
            } catch (e) {
                console.log('ROS not available, running in offline mode');
                statusText.textContent = 'Offline Mode';
                startIdleAnimations();
            }
        }
        
        initROS();
        
        // Petting (now with RL reward)
        petZone.addEventListener('click', handlePet);
        petZone.addEventListener('touchstart', (e) => {
            e.preventDefault();
            handlePet(e);
        });
        
        function handlePet(e) {
            if (isStreaming) return;
            
            // RL Reward logic
            if (latestResponseId) {
                positiveRewardCount++;
                const intensity = Math.min(positiveRewardCount, 5);
                showRewardFeedback(1, intensity);
                sendRLReward('positive', intensity, latestResponseId);
            }
            
            petCount++;
            isBeingPetted = true;
            clearTimeout(petTimeout);
            
            createRipple(e);
            
            for (let i = 0; i < 3; i++) {
                setTimeout(() => createPetHeart(e, i), i * 100);
            }
            
            petCounter.textContent = `Pets: ${petCount}`;
            petCounter.classList.add('show');
            
            setEmotion('petted', '💖');
            
            if (petCount % 3 === 0) {
                const response = petResponses[Math.floor(Math.random() * petResponses.length)];
                speechBubble.textContent = response;
                speechBubble.classList.add('show');
                createMessageElement('grace', response);
            } else {
                speechBubble.textContent = ['💖', '💕', '💗', '🥰', '💝'][Math.floor(Math.random() * 5)];
                speechBubble.classList.add('show');
            }
            
            robot.classList.add('petting');
            
            petTimeout = setTimeout(() => {
                isBeingPetted = false;
                robot.classList.remove('petting');
                speechBubble.classList.remove('show');
                petCounter.classList.remove('show');
                setEmotion('happy');
            }, 2000);
        }
        
        function createRipple(e) {
            const rect = robot.getBoundingClientRect();
            const x = e.clientX ? e.clientX - rect.left : rect.width / 2;
            const y = e.clientY ? e.clientY - rect.top : rect.height / 2;
            
            const ripple = document.createElement('div');
            ripple.className = 'ripple';
            ripple.style.width = '20px';
            ripple.style.height = '20px';
            ripple.style.left = (x - 10) + 'px';
            ripple.style.top = (y - 10) + 'px';
            
            robot.appendChild(ripple);
            setTimeout(() => ripple.remove(), 600);
        }
        
        function createPetHeart(e, index) {
            const rect = robot.getBoundingClientRect();
            const heart = document.createElement('div');
            heart.className = 'pet-heart';
            heart.innerHTML = ['💖', '💕', '💗', '💝', '💘'][Math.floor(Math.random() * 5)];
            
            const offsetX = e.clientX ? (e.clientX - rect.left) : rect.width / 2;
            const offsetY = e.clientY ? (e.clientY - rect.top) : rect.height / 2;
            
            heart.style.left = (offsetX + (Math.random() - 0.5) * 40) + 'px';
            heart.style.top = (offsetY + (Math.random() - 0.5) * 40) + 'px';
            heart.style.animationDelay = (index * 0.1) + 's';
            
            robot.appendChild(heart);
            setTimeout(() => heart.remove(), 1500);
        }
        
        // Emotion detection
        function detectAndSetEmotion(text) {
            const emojiRegex = /(\p{Emoji})/u;
            const match = text.match(emojiRegex);
            
            if (match) {
                const emoji = match[1];
                const emotion = emotionMap[emoji] || 'happy';
                setEmotion(emotion, emoji);
            } else {
                setEmotion('happy');
            }
        }
        
        // Set emotion
        function setEmotion(emotion, emoji = null) {
            if (isBeingPetted && emotion !== 'petted') return;
            
            currentEmotion = emotion;
            
            robotHead.classList.remove('happy', 'sad', 'angry', 'surprised', 'thinking', 'loving', 'petted');
            leftEye.classList.remove('happy', 'sad', 'angry', 'surprised', 'love', 'look-left', 'look-right', 'look-up');
            rightEye.classList.remove('happy', 'sad', 'angry', 'surprised', 'love', 'look-left', 'look-right', 'look-up');
            mouth.classList.remove('smile', 'open', 'o-shape', 'line', 'sad', 'angry', 'love', 'petted');
            
            robotHead.classList.add(emotion);
            
            if (emoji) {
                emotionIndicator.textContent = emoji;
                emotionIndicator.classList.add('show');
                setTimeout(() => emotionIndicator.classList.remove('show'), 2000);
            }
            
            switch(emotion) {
                case 'happy':
                    leftEye.classList.add('happy');
                    rightEye.classList.add('happy');
                    mouth.classList.add('smile');
                    break;
                case 'sad':
                    leftEye.classList.add('sad');
                    rightEye.classList.add('sad');
                    mouth.classList.add('sad');
                    break;
                case 'angry':
                    leftEye.classList.add('angry');
                    rightEye.classList.add('angry');
                    mouth.classList.add('angry');
                    break;
                case 'surprised':
                    leftEye.classList.add('surprised');
                    rightEye.classList.add('surprised');
                    mouth.classList.add('o-shape');
                    break;
                case 'thinking':
                    leftEye.classList.add('look-up');
                    rightEye.classList.add('look-up');
                    mouth.classList.add('line');
                    break;
                case 'loving':
                    leftEye.classList.add('love');
                    rightEye.classList.add('love');
                    mouth.classList.add('love');
                    break;
                case 'petted':
                    leftEye.classList.add('happy');
                    rightEye.classList.add('happy');
                    mouth.classList.add('petted');
                    break;
                default:
                    leftEye.classList.add('happy');
                    rightEye.classList.add('happy');
                    mouth.classList.add('smile');
            }
        }
        
        // Streaming
        function startStreaming(firstChunk) {
            if (isStreaming) return;
            
            isStreaming = true;
            isSpeaking = true;
            streamBuffer = firstChunk;
            
            userInput.disabled = true;
            sendBtn.disabled = true;
            
            speechBubble.textContent = 'Thinking... 💭';
            speechBubble.classList.add('show');
            
            currentStreamMessage = createMessageElement('grace', '');
            const contentDiv = currentStreamMessage.querySelector('.message-content');
            
            contentDiv.textContent = streamBuffer;
            
            const cursor = document.createElement('span');
            cursor.className = 'streaming-cursor';
            contentDiv.appendChild(cursor);
            
            startSpeakingAnimation();
            
            if (streamBuffer.length > 15) {
                const preview = streamBuffer.substring(0, 20);
                speechBubble.textContent = preview + (streamBuffer.length > 20 ? '...' : '');
            }
        }
        
        function appendToStream(delta) {
            if (!isStreaming || !currentStreamMessage) return;
            
            streamBuffer += delta;
            
            const contentDiv = currentStreamMessage.querySelector('.message-content');
            const cursor = contentDiv.querySelector('.streaming-cursor');
            
            contentDiv.textContent = streamBuffer;
            
            if (cursor) {
                contentDiv.appendChild(cursor);
            } else {
                const newCursor = document.createElement('span');
                newCursor.className = 'streaming-cursor';
                contentDiv.appendChild(newCursor);
            }
            
            if (streamBuffer.length % 10 === 0 && streamBuffer.length > 15) {
                const preview = streamBuffer.substring(0, 20);
                speechBubble.textContent = preview + (streamBuffer.length > 20 ? '...' : '');
            }
            
            if (streamBuffer.length % 5 === 0) {
                randomizeMouth();
            }
            
            chatContainer.scrollTop = chatContainer.scrollHeight;
        }
        
        function finishStreaming() {
            isStreaming = false;
            isSpeaking = false;
            
            if (currentStreamMessage) {
                const cursor = currentStreamMessage.querySelector('.streaming-cursor');
                if (cursor) cursor.remove();
            }
            
            userInput.disabled = false;
            sendBtn.disabled = false;
            userInput.focus();
            
            stopSpeakingAnimation();
            
            speechBubble.textContent = 'Done! 💕';
            setTimeout(() => {
                speechBubble.classList.remove('show');
            }, 1500);
            
            chatContainer.scrollTop = chatContainer.scrollHeight;
            currentStreamMessage = null;
            streamBuffer = '';
        }
        
        function createMessageElement(sender, text) {
            const msgDiv = document.createElement('div');
            msgDiv.className = `message ${sender}`;
            
            const label = document.createElement('div');
            label.className = 'message-label';
            label.textContent = sender === 'user' ? 'You 💙' : 'Grace 💕';
            
            const content = document.createElement('div');
            content.className = 'message-content';
            content.textContent = text;
            
            msgDiv.appendChild(label);
            msgDiv.appendChild(content);
            chatContainer.appendChild(msgDiv);
            chatContainer.scrollTop = chatContainer.scrollHeight;
            
            return msgDiv;
        }
        
        function randomizeMouth() {
            const mouths = ['open', 'o-shape', 'smile'];
            const random = mouths[Math.floor(Math.random() * mouths.length)];
            mouth.className = 'mouth ' + random;
        }
        
        function startSpeakingAnimation() {
            speechBubble.classList.add('show', 'speaking');
            speakingWaves.classList.add('active');
        }
        
        function stopSpeakingAnimation() {
            speechBubble.classList.remove('show', 'speaking');
            speakingWaves.classList.remove('active');
            setEmotion(currentEmotion);
        }
        
        // Idle animations
        function startIdleAnimations() {
            idleInterval = setInterval(() => {
                if (!isSpeaking && !isStreaming && !isBeingPetted) {
                    const expressions = ['happy', 'look-left', 'look-right', 'look-up', 'normal'];
                    const randomExp = expressions[Math.floor(Math.random() * expressions.length)];
                    if (randomExp !== 'normal') {
                        leftEye.classList.add(randomExp);
                        rightEye.classList.add(randomExp);
                    }
                    setTimeout(() => {
                        leftEye.classList.remove('look-left', 'look-right', 'look-up');
                        rightEye.classList.remove('look-left', 'look-right', 'look-up');
                    }, 1000);
                }
            }, 3000);
            
            blinkInterval = setInterval(() => {
                if (!isSpeaking && !isStreaming && !isBeingPetted) {
                    blink();
                }
            }, 4000);
        }
        
        function stopIdleAnimations() {
            clearInterval(idleInterval);
            clearInterval(blinkInterval);
        }
        
        function blink() {
            leftEye.classList.add('blink');
            rightEye.classList.add('blink');
            setTimeout(() => {
                leftEye.classList.remove('blink');
                rightEye.classList.remove('blink');
            }, 200);
        }
        
        