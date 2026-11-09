// Matrix Rain
        const canvas = document.getElementById('matrixCanvas');
        const ctx = canvas.getContext('2d');

        function resizeCanvas() {
            canvas.width = window.innerWidth;
            canvas.height = window.innerHeight;
            initDrops();
        }

        const chars = 'アイウエオカキクケコサシスセソタチツテトナニヌネノハヒフヘホマミムメモヤユヨラリルレロワヲン0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz@#$%^&*';
        const charArray = chars.split('');

        const fontSize = 16;
        let columns = 0;
        let drops = [];

        function initDrops() {
            columns = Math.floor(canvas.width / fontSize);
            drops = [];
            for (let i = 0; i < columns; i++) {
                drops[i] = Math.random() * -50;
            }
        }

        resizeCanvas();
        window.addEventListener('resize', resizeCanvas);

        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        function drawMatrix() {
            ctx.fillStyle = 'rgba(0, 0, 0, 0.06)';
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            ctx.fillStyle = '#00ff88';
            ctx.font = fontSize + 'px monospace';
            ctx.shadowBlur = 8;
            ctx.shadowColor = '#00ff88';

            for (let i = 0; i < drops.length; i++) {
                const text = charArray[Math.floor(Math.random() * charArray.length)];
                const x = i * fontSize;
                const y = drops[i] * fontSize;

                ctx.fillText(text, x, y);

                if (y > canvas.height && Math.random() > 0.975) {
                    drops[i] = 0;
                }
                drops[i]++;
            }

            ctx.shadowBlur = 0;
        }

        function drawMatrixHeads() {
            for (let i = 0; i < drops.length; i++) {
                if (Math.random() > 0.98) {
                    const text = charArray[Math.floor(Math.random() * charArray.length)];
                    const x = i * fontSize;
                    const y = (drops[i] - 1) * fontSize;

                    ctx.fillStyle = '#ccffcc';
                    ctx.shadowBlur = 15;
                    ctx.shadowColor = '#00ff88';
                    ctx.fillText(text, x, y);
                }
            }
            ctx.shadowBlur = 0;
        }

        let matrixTimer = null;
        function syncMatrixAnimation() {
            const shouldAnimate = document.documentElement.dataset.theme === 'dark';
            if (shouldAnimate && matrixTimer === null) {
                matrixTimer = setInterval(() => {
                    drawMatrix();
                    drawMatrixHeads();
                }, 70);
            } else if (!shouldAnimate && matrixTimer !== null) {
                clearInterval(matrixTimer);
                matrixTimer = null;
            }
        }
        new MutationObserver(syncMatrixAnimation).observe(document.documentElement, {
            attributes: true,
            attributeFilter: ['data-theme'],
        });
        syncMatrixAnimation();

        // Circuit elements
        function createCircuitElements() {
            const bg = document.getElementById('cyberBg');

            for (let i = 0; i < 20; i++) {
                const node = document.createElement('div');
                node.className = 'circuit-node';
                node.style.left = Math.random() * 100 + '%';
                node.style.top = Math.random() * 100 + '%';
                node.style.animationDelay = Math.random() * 2 + 's';
                bg.appendChild(node);
            }

            for (let i = 0; i < 10; i++) {
                const line = document.createElement('div');
                line.className = 'data-line';
                line.style.top = Math.random() * 100 + '%';
                line.style.width = (100 + Math.random() * 200) + 'px';
                line.style.animationDelay = Math.random() * 3 + 's';
                line.style.animationDuration = (2 + Math.random() * 2) + 's';
                bg.appendChild(line);
            }

            const symbols = ['◈', '◉', '◆', '▣', '◊', '⚡', '◐', '◑', '◒', '◓', '▶', '◀', '▼', '▲'];
            for (let i = 0; i < 15; i++) {
                const symbol = document.createElement('div');
                symbol.className = 'circuit-symbol';
                symbol.textContent = symbols[Math.floor(Math.random() * symbols.length)];
                symbol.style.left = Math.random() * 100 + '%';
                symbol.style.top = Math.random() * 100 + '%';
                symbol.style.animationDelay = Math.random() * 10 + 's';
                symbol.style.animationDuration = (8 + Math.random() * 6) + 's';
                bg.appendChild(symbol);
            }
        }
        createCircuitElements();

        // Show thumb up/down animation with number
        function showThumbAnimation(type, count, event) {
            const thumb = document.createElement('div');
            thumb.className = `thumb-animation ${type}`;
            thumb.innerHTML = type === 'up' ? `👍 +${count}` : `👎 -${count}`;

            // Position near the click/touch
            const rect = robot.getBoundingClientRect();
            const x = event && event.clientX ? event.clientX - rect.left : rect.width / 2;
            const y = event && event.clientY ? event.clientY - rect.top : rect.height / 2;

            // Add some randomness to position
            const randomX = (Math.random() - 0.5) * 60;
            thumb.style.left = (x + randomX) + 'px';
            thumb.style.top = (y - 20) + 'px';

            robot.appendChild(thumb);
            setTimeout(() => thumb.remove(), 1000);
        }

        // Robot elements
        const robot = document.getElementById('robot');
        const petZone = document.getElementById('petZone');
        const pokeZoneLeft = document.getElementById('pokeZoneLeft');
        const pokeZoneRight = document.getElementById('pokeZoneRight');
        const robotHead = document.getElementById('robotHead');
        const leftEye = document.getElementById('leftEye');
        const rightEye = document.getElementById('rightEye');
        const mouth = document.getElementById('mouth');
        const speechBubble = document.getElementById('speechBubble');
        const speakingWaves = document.getElementById('speakingWaves');
        const emotionIndicator = document.getElementById('emotionIndicator');
        const petCounter = document.getElementById('petCounter');
        const pokeCounter = document.getElementById('pokeCounter');
        const chatContainer = document.getElementById('chatContainer');
        const statusDot = document.getElementById('statusDot');
        const statusText = document.getElementById('statusText');
        const userInput = document.getElementById('userInput');
        const sendBtn = document.getElementById('sendBtn');
        const plusMenu = document.getElementById('plusMenu');
        const globeBtn = document.getElementById('globeBtn');
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
        let pokeCount = 0;
        let isBeingPetted = false;
        let isBeingPoked = false;
        let petTimeout;
        let pokeTimeout;
        let ros = null;
        let isROSConnected = false;
        window.agiRosConnected = false;
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

        const pokeResponses = [
            "😢: Ouch! That hurt...",
            "💔: I'll try to do better...",
            "😞: Sorry if I messed up...",
            "🥺: *sad beep boop*",
            "😿: Please don't be mad..."
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
                if (!isBeingPetted && !isBeingPoked) setEmotion('happy');
            }, 2000);
        }

        // RL Reward Functions
        function generateResponseId() {
            return 'resp_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
        }

        function showRewardFeedback(type, intensity) {
            rewardFeedback.className = 'reward-feedback ' + (type > 0 ? 'positive' : 'negative');
            rewardFeedback.textContent = type > 0 ? '👍 +' + intensity : '👎 -' + intensity;
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

        // POKE HANDLER (replaces angry robot button)
        function handlePoke(e) {
            e.stopPropagation(); // Don't trigger petting

            if (isStreaming) return;

            // Allow poking even in offline mode for feedback fun
            if (!latestResponseId) {
                // Just show the poke animation without sending RL reward
                console.log('Poke in offline mode - no RL reward sent');
            }

            negativeRewardCount++;
            pokeCount++;
            const intensity = Math.min(negativeRewardCount, 5);

            // Visual feedback
            showRewardFeedback(-1, intensity);

            // Update counter
            pokeCounter.textContent = `Penalties: ${pokeCount}`;
            pokeCounter.classList.add('show');

            // Show thumb down animation
            showThumbAnimation('down', pokeCount, e);

            // Create poke effect
            createPokeEffect(e);

            // Robot reaction
            isBeingPoked = true;
            clearTimeout(pokeTimeout);

            robot.classList.add('poking');
            setEmotion('poked', '😢');

            // Red mark on poked cheek
            const cheekSide = e.target.id === 'pokeZoneLeft' ? 'left' : 'right';
            const cheek = document.getElementById(cheekSide + 'Cheek');
            if (cheek) {
                cheek.style.background = 'rgba(255, 107, 107, 0.9)';
                cheek.style.transform = 'scale(1.3)';
                setTimeout(() => {
                    cheek.style.background = '';
                    cheek.style.transform = '';
                }, 1000);
            }

            // Send to ROS
            sendRLReward('negative', intensity, latestResponseId);

            // Response
            if (pokeCount % 2 === 0) {
                const response = pokeResponses[Math.floor(Math.random() * pokeResponses.length)];
                speechBubble.textContent = response;
                speechBubble.classList.add('show');
                createMessageElement('grace', response);
            } else {
                speechBubble.textContent = ['😢', '💔', '😞', '🥺'][Math.min(intensity - 1, 3)] + ' Ouch!';
                speechBubble.classList.add('show');
            }

            pokeTimeout = setTimeout(() => {
                isBeingPoked = false;
                robot.classList.remove('poking');
                speechBubble.classList.remove('show');
                pokeCounter.classList.remove('show');
                setEmotion('happy');
            }, 2000);

            // Reset after delay
            setTimeout(() => {
                negativeRewardCount = 0;
            }, 5000);
        }

        function createPokeEffect(e) {
            const rect = robot.getBoundingClientRect();
            const x = e.clientX ? e.clientX - rect.left : rect.width / 2;
            const y = e.clientY ? e.clientY - rect.top : rect.height / 2;

            // Ripple
            const ripple = document.createElement('div');
            ripple.className = 'ripple poke';
            ripple.style.width = '15px';
            ripple.style.height = '15px';
            ripple.style.left = (x - 7) + 'px';
            ripple.style.top = (y - 7) + 'px';
            robot.appendChild(ripple);
            setTimeout(() => ripple.remove(), 600);

            // Poke mark
            const mark = document.createElement('div');
            mark.className = 'poke-mark';
            mark.innerHTML = '💢';
            mark.style.left = (x - 10) + 'px';
            mark.style.top = (y - 20) + 'px';
            robot.appendChild(mark);
            setTimeout(() => mark.remove(), 1000);
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
                currentImage = event.target.result
                previewImg.src = currentImage;
                imagePreviewArea.classList.add('show');
                imagePrompt.focus();
                plusMenu.classList.remove('show');

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

        // ROS Initialization
        function initROS() {
            try {
                const serverHost = window.location.hostname;
                const rosScheme = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
                const rosUrl = `${rosScheme}//${window.location.host}/rosbridge`;

                console.log(`🤖 Connecting to ROS at: ${rosUrl}`);
                statusText.textContent = `Connecting to ${serverHost}...`;

                ros = new ROSLIB.Ros({ url: rosUrl });

                ros.on('connection', () => {
                    isROSConnected = true;
                    window.agiRosConnected = true;
                    window.dispatchEvent(new CustomEvent('agi-ros-presence', { detail: { online: true } }));
                    statusDot.classList.add('connected');
                    statusText.textContent = 'Connected';
                    speechBubble.textContent = 'Connected! 💕';
                    speechBubble.classList.add('show');
                    setTimeout(() => speechBubble.classList.remove('show'), 2000);
                    startIdleAnimations();
                });

                ros.on('error', (error) => {
                    console.error('ROS connection error:', error);
                    isROSConnected = false;
                    window.agiRosConnected = false;
                    window.dispatchEvent(new CustomEvent('agi-ros-presence', { detail: { online: false } }));
                    statusText.textContent = 'Offline Mode';
                    statusDot.classList.remove('connected');
                    speechBubble.textContent = 'Hello! 💕';
                    speechBubble.classList.add('show');
                    setTimeout(() => speechBubble.classList.remove('show'), 3000);
                    startIdleAnimations();
                });

                ros.on('close', () => {
                    isROSConnected = false;
                    window.agiRosConnected = false;
                    window.dispatchEvent(new CustomEvent('agi-ros-presence', { detail: { online: false } }));
                    statusDot.classList.remove('connected');
                    statusText.textContent = 'Reconnecting...';
                    stopIdleAnimations();

                    setTimeout(() => {
                        console.log('🔄 Attempting reconnect...');
                        initROS();
                    }, 5000);
                });

                imageInputTopic = new ROSLIB.Topic({
                    ros: ros,
                    name: '/cns/image_input',
                    messageType: 'std_msgs/String'
                });

                rlRewardTopic = new ROSLIB.Topic({
                    ros: ros,
                    name: '/cns/rl_reward',
                    messageType: 'std_msgs/String'
                });

                const textResponseTopic = new ROSLIB.Topic({
                    ros: ros,
                    name: '/tms/text_response',
                    messageType: 'std_msgs/String'
                });

                textResponseTopic.subscribe((msg) => {
                    try {
                        const data = JSON.parse(msg.data);
                        if (data.type === 'start') {
                            detectAndSetEmotion(data.content);
                            startStreaming(data.content);
                        } else if (data.type === 'delta') {
                            appendToStream(data.content);
                        } else if (data.type === 'done') {
                            finishStreaming();
                            latestResponseId = data.radio_id || generateResponseId();
                            positiveRewardCount = 0;
                            negativeRewardCount = 0;
                            pokeCount = 0;
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

                window.imageInputTopic = imageInputTopic;
                window.rlRewardTopic = rlRewardTopic;

            } catch (e) {
                console.log('ROS not available, running in offline mode:', e);
                statusText.textContent = 'Offline Mode';
                startIdleAnimations();
            }
        }

        initROS();

        // Petting (positive reward)
        petZone.addEventListener('click', handlePet);
        petZone.addEventListener('touchstart', (e) => {
            e.preventDefault();
            handlePet(e);
        });

        // Poking (negative reward) - both cheeks
        pokeZoneLeft.addEventListener('click', handlePoke);
        pokeZoneRight.addEventListener('click', handlePoke);
        pokeZoneLeft.addEventListener('touchstart', (e) => {
            e.preventDefault();
            e.stopPropagation();
            handlePoke(e);
        });
        pokeZoneRight.addEventListener('touchstart', (e) => {
            e.preventDefault();
            e.stopPropagation();
            handlePoke(e);
        });

        function handlePet(e) {
            if (isStreaming) return;

            if (latestResponseId) {
                positiveRewardCount++;
                const intensity = Math.min(positiveRewardCount, 5);
                showRewardFeedback(1, intensity);
                sendRLReward('positive', intensity, latestResponseId);
            }

            petCount++;
            isBeingPetted = true;
            clearTimeout(petTimeout);

            // Show thumb up animation
            showThumbAnimation('up', petCount, e);

            createRipple(e);

            for (let i = 0; i < 3; i++) {
                setTimeout(() => createPetHeart(e, i), i * 100);
            }

            petCounter.textContent = `Reward: ${petCount}`;
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
            if (isBeingPoked && emotion !== 'poked') return;

            currentEmotion = emotion;

            robotHead.classList.remove('happy', 'sad', 'angry', 'surprised', 'thinking', 'loving', 'petted', 'poked');
            leftEye.classList.remove('happy', 'sad', 'angry', 'surprised', 'love', 'poked', 'look-left', 'look-right', 'look-up');
            rightEye.classList.remove('happy', 'sad', 'angry', 'surprised', 'love', 'poked', 'look-left', 'look-right', 'look-up');
            mouth.classList.remove('smile', 'open', 'o-shape', 'line', 'sad', 'angry', 'love', 'petted', 'surprised', 'poked');

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
                    mouth.classList.add('surprised');
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
                case 'poked':
                    leftEye.classList.add('poked');
                    rightEye.classList.add('poked');
                    mouth.classList.add('poked');
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

            if (sender === 'grace') {
                const avatar = document.createElement('div');
                avatar.className = 'grace-mini-avatar';
                avatar.setAttribute('aria-hidden', 'true');
                avatar.textContent = '✦';
                msgDiv.appendChild(avatar);
            }


            const label = document.createElement('div');
            label.className = 'message-label';
            label.textContent = sender === 'user' ? 'You' : 'Grace';

            const content = document.createElement('div');
            content.className = 'message-content';
            content.textContent = text;

            msgDiv.appendChild(label);
            msgDiv.appendChild(content);
            const timestamp = document.createElement('time');
            const now = new Date();
            timestamp.className = 'message-time';
            timestamp.dateTime = now.toISOString();
            timestamp.textContent = now.toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' });
            msgDiv.appendChild(timestamp);
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
                if (!isSpeaking && !isStreaming && !isBeingPetted && !isBeingPoked) {
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
                if (!isSpeaking && !isStreaming && !isBeingPetted && !isBeingPoked) {
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

        // Compress image before sending
        function compressImage(dataUrl, callback) {
            const img = new Image();
            img.onload = function() {
                let width = img.width;
                let height = img.height;

                const MAX_SIZE = 1024;
                if (width > height) {
                    if (width > MAX_SIZE) { height *= MAX_SIZE / width; width = MAX_SIZE; }
                } else {
                    if (height > MAX_SIZE) { width *= MAX_SIZE / height; height = MAX_SIZE; }
                }

                const canvas = document.createElement('canvas');
                canvas.width = width;
                canvas.height = height;
                canvas.getContext('2d').drawImage(img, 0, 0, width, height);

                const compressed = canvas.toDataURL('image/jpeg', 0.7);
                console.log(`Compressed: ${dataUrl.length} → ${compressed.length}`);
                callback(compressed);
            };
            img.src = dataUrl;
        }

        // Send message
        async function publishRadioInput(text) {
            const search = webSearchEnabled ? 'true' : 'false';
            const response = await fetch(`/api/telepathic/send?web_search=${search}`, {
                method: 'POST',
                headers: { 'Content-Type': 'text/plain; charset=utf-8' },
                body: text
            });
            if (!response.ok) {
                const body = await response.json().catch(() => ({}));
                const detail = body.detail;
                throw new Error(detail || `Encoder returned HTTP ${response.status}`);
            }
            return response.json();
        }

        function sendSignal() {
            const val = userInput.value.trim();
            if (isStreaming) return;

            if (currentImage) {
                const promptText = imagePrompt.value.trim() || "What's in this image?";

                if (!isROSConnected || !window.imageInputTopic) {
                    speechBubble.textContent = 'Not connected! Check ROS...';
                    speechBubble.classList.add('show');
                    setTimeout(() => speechBubble.classList.remove('show'), 3000);
                    return;
                }

                compressImage(currentImage, function(compressed) {
                    const base64 = compressed.split(',')[1];

                    if (base64.length > 900000) {
                        speechBubble.textContent = 'Image too large! Try smaller.';
                        speechBubble.classList.add('show');
                        setTimeout(() => speechBubble.classList.remove('show'), 3000);
                        return;
                    }

                    createMessageElement('user', `📷 ${promptText}`);

                    const imageData = {
                        prompt: promptText,
                        image_base64: base64,
                        web_search: webSearchEnabled
                    };

                    try {
                        window.imageInputTopic.publish(new ROSLIB.Message({
                            data: JSON.stringify(imageData)
                        }));
                        console.log('✅ Image sent:', base64.length, 'chars');
                        setEmotion('thinking', '🤔');
                        removeImage();
                        createFloatingHeart();
                    } catch (e) {
                        console.error('❌ Send failed:', e);
                    }
                });

            } else if (val) {
                createMessageElement('user', val);

                if (isROSConnected) {
                    publishRadioInput(val).then(() => {
                        console.log('📡 TelepathicSignal sent through ROS:', val, '| Web search:', webSearchEnabled ? 'ON' : 'OFF');
                    }).catch((error) => {
                        console.error('TelepathicSignal encoding or publish failed:', error);
                        speechBubble.textContent = 'Message could not be sent. Please try again.';
                        speechBubble.classList.add('show');
                        setTimeout(() => speechBubble.classList.remove('show'), 3000);
                    });
                } else {
                    setTimeout(() => {
                        const offlineResponse = "💖: I'm running in offline mode, but I still love you! Pet me instead? 💕";
                        detectAndSetEmotion(offlineResponse);
                        createMessageElement('grace', offlineResponse);
                    }, 500);
                }

                setEmotion('surprised', '✨');
                setTimeout(() => setEmotion('happy'), 500);

                userInput.value = '';
                createFloatingHeart();
            }
        }

        function createFloatingHeart() {
            const heart = document.createElement('div');
            heart.className = 'float-hearts';
            heart.innerHTML = ['💖', '💕', '💗', '💝', '💘'][Math.floor(Math.random() * 5)];
            heart.style.left = Math.random() * window.innerWidth + 'px';
            heart.style.animationDuration = (3 + Math.random() * 2) + 's';
            document.body.appendChild(heart);

            setTimeout(() => heart.remove(), 5000);
        }

        // Event listeners
        userInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') sendSignal();
        });

        imagePrompt.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') sendSignal();
        });

        // Initial state
        setEmotion('happy');

        // Test functions
        window.testEmotion = function(emotion) {
            const testEmojis = {
                'happy': '😊',
                'sad': '😢',
                'angry': '😠',
                'surprised': '😮',
                'thinking': '🤔',
                'loving': '❤️',
                'petted': '💖',
                'poked': '😢'
            };
            setEmotion(emotion, testEmojis[emotion]);
        };

        window.testPet = function() {
            const fakeEvent = { clientX: null, clientY: null };
            handlePet(fakeEvent);
        };

        window.testPoke = function() {
            const fakeEvent = { clientX: null, clientY: null, target: { id: 'pokeZoneLeft' } };
            handlePoke(fakeEvent);
        };

        window.toggleWebSearch = toggleWebSearch;
