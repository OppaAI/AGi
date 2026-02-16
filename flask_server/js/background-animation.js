// Matrix Rain (same as before)
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
        
        setInterval(() => {
            drawMatrix();
            drawMatrixHeads();
        }, 33);
        
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
        
        