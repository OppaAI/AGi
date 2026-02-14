// js/app.js - Main initialization and Matrix rain background
const App = {
    canvas: null,
    ctx: null,
    chars: 'アイウエオカキクケコサシスセソタチツテトナニヌネノハヒフヘホマミムメモヤユヨラリルレロワヲン0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz@#$%^&*',
    charArray: [],
    fontSize: 16,
    columns: 0,
    drops: [],

    init() {
        this.initMatrix();
        this.createCircuitElements();
        this.initModules();
        this.bindGlobalEvents();
    },

    initMatrix() {
        this.canvas = document.getElementById('matrixCanvas');
        this.ctx = this.canvas.getContext('2d');
        this.charArray = this.chars.split('');
        
        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());
        
        this.ctx.fillStyle = '#000';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        
        setInterval(() => {
            this.drawMatrix();
            this.drawMatrixHeads();
        }, 33);
    },

    resizeCanvas() {
        this.canvas.width = window.innerWidth;
        this.canvas.height = window.innerHeight;
        this.initDrops();
    },

    initDrops() {
        this.columns = Math.floor(this.canvas.width / this.fontSize);
        this.drops = [];
        for (let i = 0; i < this.columns; i++) {
            this.drops[i] = Math.random() * -50;
        }
    },

    drawMatrix() {
        this.ctx.fillStyle = 'rgba(0, 0, 0, 0.06)';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        
        this.ctx.fillStyle = '#00ff88';
        this.ctx.font = this.fontSize + 'px monospace';
        this.ctx.shadowBlur = 8;
        this.ctx.shadowColor = '#00ff88';
        
        for (let i = 0; i < this.drops.length; i++) {
            const text = this.charArray[Math.floor(Math.random() * this.charArray.length)];
            const x = i * this.fontSize;
            const y = this.drops[i] * this.fontSize;
            
            this.ctx.fillText(text, x, y);
            
            if (y > this.canvas.height && Math.random() > 0.975) {
                this.drops[i] = 0;
            }
            this.drops[i]++;
        }
        
        this.ctx.shadowBlur = 0;
    },

    drawMatrixHeads() {
        for (let i = 0; i < this.drops.length; i++) {
            if (Math.random() > 0.98) {
                const text = this.charArray[Math.floor(Math.random() * this.charArray.length)];
                const x = i * this.fontSize;
                const y = (this.drops[i] - 1) * this.fontSize;
                
                this.ctx.fillStyle = '#ccffcc';
                this.ctx.shadowBlur = 15;
                this.ctx.shadowColor = '#00ff88';
                this.ctx.fillText(text, x, y);
            }
        }
        this.ctx.shadowBlur = 0;
    },

    createCircuitElements() {
        const bg = document.getElementById('cyberBg');
        
        // Create nodes
        for (let i = 0; i < 20; i++) {
            const node = document.createElement('div');
            node.className = 'circuit-node';
            node.style.left = Math.random() * 100 + '%';
            node.style.top = Math.random() * 100 + '%';
            node.style.animationDelay = Math.random() * 2 + 's';
            bg.appendChild(node);
        }
        
        // Create data lines
        for (let i = 0; i < 10; i++) {
            const line = document.createElement('div');
            line.className = 'data-line';
            line.style.top = Math.random() * 100 + '%';
            line.style.width = (100 + Math.random() * 200) + 'px';
            line.style.animationDelay = Math.random() * 3 + 's';
            line.style.animationDuration = (2 + Math.random() * 2) + 's';
            bg.appendChild(line);
        }
        
        // Create symbols
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
    },

    initModules() {
        // Initialize all modules
        Robot.init();
        Chat.init();
        Rewards.init();
        
        // Initial robot state
        Robot.setEmotion('happy');
    },

    bindGlobalEvents() {
        // Expose test functions globally
        window.testEmotion = (emotion) => {
            const testEmojis = {
                'happy': '😊',
                'sad': '😢',
                'angry': '😠',
                'surprised': '😮',
                'thinking': '🤔',
                'loving': '❤️',
                'petted': '💖'
            };
            Robot.setEmotion(emotion, testEmojis[emotion]);
        };

        window.testPet = () => {
            const fakeEvent = { clientX: null, clientY: null };
            Robot.handlePet(fakeEvent);
        };

        // Expose necessary functions for HTML onclick handlers
        window.togglePlusMenu = () => Chat.togglePlusMenu();
        window.toggleWebSearch = () => Chat.toggleWebSearch();
        window.handleNegativeReward = () => Rewards.handleNegativeReward();
        window.handleImageSelect = (input) => Chat.handleImageSelect(input);
        window.removeImage = () => Chat.removeImage();
        window.sendSignal = () => Chat.sendSignal();
    }
};

// Start the app when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    App.init();
});