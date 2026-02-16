"""
Alternative RAG Implementation for Grace (No sqlite-vec required)

This is a pure Python fallback that works on any system without compiled extensions.
Uses numpy for vector operations instead of sqlite-vec.

Performance: Slightly slower for large datasets, but perfectly fine for Grace's use case.
"""

import sqlite3
import numpy as np
from sentence_transformers import SentenceTransformer
import json
from datetime import datetime


class SimpleSQLiteRAG:
    """
    Lightweight vector RAG using SQLite (without sqlite-vec extension)
    
    Features:
    - Pure Python implementation (no compiled extensions)
    - Uses numpy for vector operations
    - Slightly slower but more compatible
    - Perfect for Jetson Orin Nano
    """
    
    def __init__(self, db_path: str, logger):
        self.logger = logger
        self.db_path = db_path
        
        # Initialize embedding model
        self.logger.info("Loading embedding model...")
        self.embedder = SentenceTransformer('all-MiniLM-L6-v2')
        self.logger.info("✅ Embedding model loaded (384 dimensions)")
        
        # Connect to SQLite (no extensions needed!)
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row
        
        # Initialize tables
        self._init_tables()
        
        # 7-day window storage (in memory)
        self.daily_messages = {}
        self.weekly_journals = []
        self.monthly_summaries = []
        
        self.logger.info("✅ Simple SQLite RAG initialized (no vec extension needed)")
    
    def _init_tables(self):
        """Initialize database tables (without vector extension)"""
        
        # Daily archives table
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS daily_archives (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                date TEXT NOT NULL,
                message_count INTEGER,
                full_text TEXT,
                summary TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                UNIQUE(date)
            )
        """)
        
        # Vector embeddings table (stored as JSON instead of native vectors)
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS memory_vectors (
                archive_id INTEGER PRIMARY KEY,
                embedding TEXT,  -- JSON array instead of FLOAT vector
                FOREIGN KEY (archive_id) REFERENCES daily_archives(id)
            )
        """)
        
        # Create index for faster lookups
        self.conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_daily_archives_date 
            ON daily_archives(date)
        """)
        
        # Weekly journals
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS weekly_journals (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                week_start TEXT,
                week_end TEXT,
                summary TEXT,
                daily_reflections TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # Monthly/quarterly/yearly summaries
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS periodic_summaries (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                period_type TEXT,
                period_start TEXT,
                period_end TEXT,
                summary TEXT,
                statistics TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        self.conn.commit()
        self.logger.info("✅ Database tables initialized")
    
    def add_message_to_window(self, date_str: str, message: dict):
        """Add message to 7-day rolling window"""
        if date_str not in self.daily_messages:
            self.daily_messages[date_str] = []
        
        self.daily_messages[date_str].append(message)
    
    def get_messages_in_window(self) -> list:
        """Get all messages from 7-day window"""
        dates = sorted(self.daily_messages.keys())
        
        # Keep only last 7 days
        if len(dates) > 7:
            dates = dates[-7:]
        
        # Flatten messages
        messages = []
        for date in dates:
            messages.extend(self.daily_messages[date])
        
        return messages
    
    def get_full_day_messages(self, date_str: str) -> list:
        """Get ALL messages from a specific day"""
        return self.daily_messages.get(date_str, [])
    
    def archive_day_to_rag(self, date_str: str, daily_reflection: str = ""):
        """Archive a full day's conversations to vector database"""
        messages = self.daily_messages.get(date_str, [])
        
        if not messages:
            self.logger.info(f"No messages to archive for {date_str}")
            return
        
        try:
            # Build full text of the day
            full_text_lines = []
            for msg in messages:
                role = msg.get('role', 'unknown')
                content = msg.get('content', '')
                
                # Skip images
                if msg.get('has_image'):
                    full_text_lines.append(f"{role}: [image message]")
                else:
                    full_text_lines.append(f"{role}: {content}")
            
            full_text = "\n".join(full_text_lines)
            
            # Generate embedding
            self.logger.info(f"Generating embedding for {date_str}...")
            embedding = self.embedder.encode(full_text)
            
            # Convert numpy array to JSON string for storage
            embedding_json = json.dumps(embedding.tolist())
            
            # Store in daily_archives
            cursor = self.conn.execute("""
                INSERT OR REPLACE INTO daily_archives (date, message_count, full_text, summary)
                VALUES (?, ?, ?, ?)
            """, [date_str, len(messages), full_text, daily_reflection])
            
            archive_id = cursor.lastrowid
            
            # Store embedding as JSON
            self.conn.execute("""
                INSERT OR REPLACE INTO memory_vectors (archive_id, embedding)
                VALUES (?, ?)
            """, [archive_id, embedding_json])
            
            self.conn.commit()
            
            self.logger.info(f"✅ Archived {date_str}: {len(messages)} messages → Vector DB")
            
            # Remove from active window
            if date_str in self.daily_messages:
                del self.daily_messages[date_str]
        
        except Exception as e:
            self.logger.error(f"❌ Failed to archive {date_str}: {e}")
            self.conn.rollback()
    
    def search_memory(self, query: str, top_k: int = 5, days_back: int = None) -> list:
        """
        Search vector database for relevant memories (pure Python version)
        
        Uses numpy for cosine similarity instead of SQL vector functions
        """
        try:
            # Generate query embedding
            query_embedding = self.embedder.encode(query)
            
            # Build query with optional date filter
            sql = """
                SELECT 
                    da.id,
                    da.date,
                    da.full_text,
                    da.summary,
                    da.message_count,
                    mv.embedding
                FROM memory_vectors mv
                JOIN daily_archives da ON mv.archive_id = da.id
            """
            
            params = []
            
            if days_back:
                sql += " WHERE da.date >= date('now', ?)"
                params.append(f'-{days_back} days')
            
            results = self.conn.execute(sql, params).fetchall()
            
            if not results:
                return []
            
            # Calculate cosine similarity using numpy
            similarities = []
            
            for row in results:
                # Parse JSON embedding back to numpy array
                stored_embedding = np.array(json.loads(row['embedding']))
                
                # Cosine similarity
                similarity = self._cosine_similarity(query_embedding, stored_embedding)
                
                similarities.append({
                    'date': row['date'],
                    'text': row['full_text'][:500] + "...",
                    'summary': row['summary'],
                    'message_count': row['message_count'],
                    'similarity': round(similarity, 3)
                })
            
            # Sort by similarity (descending)
            similarities.sort(key=lambda x: x['similarity'], reverse=True)
            
            # Return top K
            top_results = similarities[:top_k]
            
            self.logger.info(f"🔍 Memory search: found {len(top_results)} relevant days")
            return top_results
        
        except Exception as e:
            self.logger.error(f"❌ Memory search failed: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def _cosine_similarity(self, vec1, vec2):
        """Calculate cosine similarity between two vectors"""
        dot_product = np.dot(vec1, vec2)
        norm1 = np.linalg.norm(vec1)
        norm2 = np.linalg.norm(vec2)
        
        if norm1 == 0 or norm2 == 0:
            return 0.0
        
        return dot_product / (norm1 * norm2)
    
    def create_weekly_journal(self, start_date: str, end_date: str, daily_reflections: list):
        """Create weekly journal from 7 daily reflections"""
        if len(daily_reflections) < 7:
            return None
        
        # Build weekly summary
        summary_lines = [f"Week of {start_date} to {end_date}\n"]
        
        for reflection in daily_reflections:
            day = reflection.get('day')
            date = reflection.get('date')
            text = reflection.get('reflection', '')
            count = reflection.get('message_count', 0)
            
            summary_lines.append(f"• Day {day} ({date}): {count} msgs - {text}")
        
        summary = "\n".join(summary_lines)
        reflections_json = json.dumps(daily_reflections)
        
        # Store in database
        self.conn.execute("""
            INSERT INTO weekly_journals (week_start, week_end, summary, daily_reflections)
            VALUES (?, ?, ?, ?)
        """, [start_date, end_date, summary, reflections_json])
        
        self.conn.commit()
        
        self.logger.info(f"📖 Created weekly journal: {start_date} to {end_date}")
        return summary
    
    def generate_periodic_report(self, period_type: str, llm_generate_func):
        """Generate monthly/quarterly/yearly report from vector database"""
        from datetime import datetime, timedelta
        
        today = datetime.now().date()
        
        if period_type == 'monthly':
            start_date = (today - timedelta(days=30)).isoformat()
            days_back = 30
        elif period_type == 'quarterly':
            start_date = (today - timedelta(days=90)).isoformat()
            days_back = 90
        elif period_type == 'yearly':
            start_date = (today - timedelta(days=365)).isoformat()
            days_back = 365
        else:
            raise ValueError(f"Unknown period type: {period_type}")
        
        end_date = today.isoformat()
        
        # Retrieve all archived days in period
        archives = self.conn.execute("""
            SELECT date, summary, message_count
            FROM daily_archives
            WHERE date >= ? AND date <= ?
            ORDER BY date
        """, [start_date, end_date]).fetchall()
        
        if not archives:
            self.logger.warn(f"No archives found for {period_type} report")
            return None
        
        # Build context for LLM
        total_messages = sum(row['message_count'] for row in archives)
        daily_summaries = [
            f"{row['date']}: {row['message_count']} msgs - {row['summary']}"
            for row in archives
        ]
        
        context = f"""{period_type.upper()} REPORT
Period: {start_date} to {end_date}
Total days: {len(archives)}
Total conversations: {total_messages}

Daily Summaries:
{chr(10).join(daily_summaries)}

Generate a {period_type} reflection covering:
1. Major themes and topics discussed
2. Notable developments or changes
3. Emotional tone and overall experience
4. Looking ahead to next {period_type}
"""
        
        # Generate summary via LLM
        self.logger.info(f"📊 Generating {period_type} report with LLM...")
        summary = llm_generate_func(context)
        
        # Store in database
        statistics = json.dumps({
            'total_days': len(archives),
            'total_messages': total_messages,
            'avg_messages_per_day': round(total_messages / len(archives), 1)
        })
        
        self.conn.execute("""
            INSERT INTO periodic_summaries (period_type, period_start, period_end, summary, statistics)
            VALUES (?, ?, ?, ?, ?)
        """, [period_type, start_date, end_date, summary, statistics])
        
        self.conn.commit()
        
        self.logger.info(f"✅ {period_type.capitalize()} report generated")
        return summary
    
    def slide_window(self):
        """Slide the 7-day window (called when day 8 arrives)"""
        dates = sorted(self.daily_messages.keys())
        
        if len(dates) > 7:
            oldest_date = dates[0]
            self.logger.info(f"🗂️ Sliding window: archiving {oldest_date}")
            return oldest_date
        
        return None
    
    def get_statistics(self) -> dict:
        """Get memory statistics"""
        stats = {}
        
        # Active window
        stats['active_days'] = len(self.daily_messages)
        stats['active_messages'] = sum(len(msgs) for msgs in self.daily_messages.values())
        
        # Archived
        result = self.conn.execute("""
            SELECT 
                COUNT(*) as total_days,
                SUM(message_count) as total_messages
            FROM daily_archives
        """).fetchone()
        
        stats['archived_days'] = result['total_days'] or 0
        stats['archived_messages'] = result['total_messages'] or 0
        
        # Journals
        stats['weekly_journals'] = self.conn.execute(
            "SELECT COUNT(*) FROM weekly_journals"
        ).fetchone()[0]
        
        stats['periodic_reports'] = self.conn.execute(
            "SELECT COUNT(*) FROM periodic_summaries"
        ).fetchone()[0]
        
        return stats
    
    def close(self):
        """Close database connection"""
        if self.conn:
            self.conn.close()
