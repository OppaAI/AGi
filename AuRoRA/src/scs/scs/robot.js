// js/robot.js - Robot animations, emotions, and petting interactions
const Robot = {
    elements: {},
    state: {
        currentEmotion: 'happy',
        isBeingPetted: false,
        petCount: 0,
        isSpeaking: false,
        isStreaming: false
    },
    intervals: {
        idle: null,
        blink: null
    },
    petTimeout: null,
    
    petResponses: [
        "💖: Aww, that feels nice!",
        "🥰: I love it when you pet me!",
        "💕: You're so sweet!",
        "✨: *happy robot noises*",
        "💗: More please!",
        "🤗: That tickles!",
        "💝: You're the best!",
        "🥺: *blushes*"
    ],
    
    emotionMap: {
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
    },

    init() {
        this.elements = {
            robot: document.getElementById('robot'),
            petZone: document.getElementById('petZone'),
            robotHead: document.getElementById('robotHead'),
            leftEye: document.getElementById('leftEye'),
            rightEye: document.getElementById('rightEye'),
            mouth: document.getElementById('mouth'),
            speechBubble: document.getElementById('speechBubble'),
            speakingWaves: document.getElementById('speakingWaves'),
            emotionIndicator: document.getElementById('emotionIndicator'),
            petCounter: document.getElementById('petCounter')
        };
        
        this.bindEvents();
        this.startIdleAnimations();
    },

    bindEvents() {
        this.elements.petZone.addEventListener('click', (e) => this.handlePet(e));
        this.elements.petZone.addEventListener('touchstart', (e) => {
            e.preventDefault();
            this.handlePet(e);
        });
    },

    handlePet(e) {
        if (this.state.isStreaming) return;
        
        // RL Reward logic
        if (window.Rewards && window.Rewards.latestResponseId) {
            window.Rewards.positiveRewardCount++;
            const intensity = Math.min(window.Rewards.positiveRewardCount, 5);
            window.Rewards.showRewardFeedback(1, intensity);
            window.Rewards.sendRLReward('positive', intensity, window.Rewards.latestResponseId);
        }

        this.state.petCount++;
        this.state.isBeingPetted = true;
        clearTimeout(this.petTimeout);

        this.createRipple(e);
        
        for (let i = 0; i < 3; i++) {
            setTimeout(() => this.createPetHeart(e, i), i * 100);
        }

        this.elements.petCounter.textContent = `Pets: ${this.state.petCount}`;
        this.elements.petCounter.classList.add('show');

        this.setEmotion('petted', '💖');

        if (this.state.petCount % 3 === 0) {
            const response = this.petResponses[Math.floor(Math.random() * this.petResponses.length)];
            this.elements.speechBubble.textContent = response;
            this.elements.speechBubble.classList.add('show');
            if (window.Chat) window.Chat.createMessageElement('grace', response);
        } else {
            this.elements.speechBubble.textContent = ['💖', '💕', '💗', '🥰', '💝'][Math.floor(Math.random() * 5)];
            this.elements.speechBubble.classList.add('show');
        }

        this.elements.robot.classList.add('petting');

        this.petTimeout = setTimeout(() => {
            this.state.isBeingPetted = false;
            this.elements.robot.classList.remove('petting');
            this.elements.speechBubble.classList.remove('show');
            this.elements.petCounter.classList.remove('show');
            this.setEmotion('happy');
        }, 2000);
    },

    createRipple(e) {
        const rect = this.elements.robot.getBoundingClientRect();
        const x = e.clientX ? e.clientX - rect.left : rect.width / 2;
        const y = e.clientY ? e.clientY - rect.top : rect.height / 2;

        const ripple = document.createElement('div');
        ripple.className = 'ripple';
        ripple.style.width = '20px';
        ripple.style.height = '20px';
        ripple.style.left = (x - 10) + 'px';
        ripple.style.top = (y - 10) + 'px';

        this.elements.robot.appendChild(ripple);
        setTimeout(() => ripple.remove(), 600);
    },

    createPetHeart(e, index) {
        const rect = this.elements.robot.getBoundingClientRect();
        const heart = document.createElement('div');
        heart.className = 'pet-heart';
        heart.innerHTML = ['💖', '💕', '💗', '💝', '💘'][Math.floor(Math.random() * 5)];

        const offsetX = e.clientX ? (e.clientX - rect.left) : rect.width / 2;
        const offsetY = e.clientY ? (e.clientY - rect.top) : rect.height / 2;

        heart.style.left = (offsetX + (Math.random() - 0.5) * 40) + 'px';
        heart.style.top = (offsetY + (Math.random() - 0.5) * 40) + 'px';
        heart.style.animationDelay = (index * 0.1) + 's';

        this.elements.robot.appendChild(heart);
        setTimeout(() => heart.remove(), 1500);
    },

    detectAndSetEmotion(text) {
        const emojiRegex = /(\p{Emoji})/u;
        const match = text.match(emojiRegex);
        
        if (match) {
            const emoji = match[1];
            const emotion = this.emotionMap[emoji] || 'happy';
            this.setEmotion(emotion, emoji);
        } else {
            this.setEmotion('happy');
        }
    },

    setEmotion(emotion, emoji = null) {
        if (this.state.isBeingPetted && emotion !== 'petted') return;
        
        this.state.currentEmotion = emotion;
        
        const { robotHead, leftEye, rightEye, mouth, emotionIndicator } = this.elements;
        
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

        const eyeStates = {
            happy: ['happy', 'happy', 'smile'],
            sad: ['sad', 'sad', 'sad'],
            angry: ['angry', 'angry', 'angry'],
            surprised: ['surprised', 'surprised', 'o-shape'],
            thinking: ['look-up', 'look-up', 'line'],
            loving: ['love', 'love', 'love'],
            petted: ['happy', 'happy', 'petted']
        };

        const [leftEyeClass, rightEyeClass, mouthClass] = eyeStates[emotion] || eyeStates.happy;
        leftEye.classList.add(leftEyeClass);
        rightEye.classList.add(rightEyeClass);
        mouth.classList.add(mouthClass);
    },

    startSpeakingAnimation() {
        this.elements.speechBubble.classList.add('show', 'speaking');
        this.elements.speakingWaves.classList.add('active');
    },

    stopSpeakingAnimation() {
        this.elements.speechBubble.classList.remove('show', 'speaking');
        this.elements.speakingWaves.classList.remove('active');
        this.setEmotion(this.state.currentEmotion);
    },

    startIdleAnimations() {
        this.intervals.idle = setInterval(() => {
            if (!this.state.isSpeaking && !this.state.isStreaming && !this.state.isBeingPetted) {
                const expressions = ['happy', 'look-left', 'look-right', 'look-up', 'normal'];
                const randomExp = expressions[Math.floor(Math.random() * expressions.length)];
                if (randomExp !== 'normal') {
                    this.elements.leftEye.classList.add(randomExp);
                    this.elements.rightEye.classList.add(randomExp);
                }
                setTimeout(() => {
                    this.elements.leftEye.classList.remove('look-left', 'look-right', 'look-up');
                    this.elements.rightEye.classList.remove('look-left', 'look-right', 'look-up');
                }, 1000);
            }
        }, 3000);

        this.intervals.blink = setInterval(() => {
            if (!this.state.isSpeaking && !this.state.isStreaming && !this.state.isBeingPetted) {
                this.blink();
            }
        }, 4000);
    },

    stopIdleAnimations() {
        clearInterval(this.intervals.idle);
        clearInterval(this.intervals.blink);
    },

    blink() {
        this.elements.leftEye.classList.add('blink');
        this.elements.rightEye.classList.add('blink');
        setTimeout(() => {
            this.elements.leftEye.classList.remove('blink');
            this.elements.rightEye.classList.remove('blink');
        }, 200);
    },

    randomizeMouth() {
        const mouths = ['open', 'o-shape', 'smile'];
        const random = mouths[Math.floor(Math.random() * mouths.length)];
        this.elements.mouth.className = 'mouth ' + random;
    },

    setSpeakingState(isSpeaking) {
        this.state.isSpeaking = isSpeaking;
    },

    setStreamingState(isStreaming) {
        this.state.isStreaming = isStreaming;
    }
};