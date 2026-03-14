#!/usr/bin/env python3
"""
Test Hugo Blog Posting for Grace

This script lets you test the Hugo blogging system before integrating into cnc.py.
You can see exactly how it works and verify your setup.

Usage:
    python3 test_hugo_blog.py
"""

import subprocess
from pathlib import Path
from datetime import datetime
import os
import sys

# Configuration (customize these)
HUGO_BLOG_DIR = os.getenv('HUGO_BLOG_DIR', str(Path.home() / 'grace-blog'))
HUGO_GIT_REMOTE = os.getenv('HUGO_GIT_REMOTE', 'origin')
HUGO_GIT_BRANCH = os.getenv('HUGO_GIT_BRANCH', 'main')


class SimpleLogger:
    """Simple logger for testing"""
    def info(self, msg): print(f"ℹ️  {msg}")
    def warn(self, msg): print(f"⚠️  {msg}")
    def error(self, msg): print(f"❌ {msg}")


class HugoBlogger:
    """Simplified Hugo blogger for testing"""
    
    def __init__(self, blog_dir: str, logger, auto_deploy: bool = True):
        self.blog_dir = Path(blog_dir)
        self.logger = logger
        self.auto_deploy = auto_deploy
        self.enabled = False
        
        # Validate blog directory
        if not self.blog_dir.exists():
            self.logger.error(f"Hugo blog directory not found: {blog_dir}")
            self.logger.info("Create with: hugo new site grace-blog")
            return
        
        # Check if it's a Hugo site
        if not (self.blog_dir / 'config.toml').exists() and not (self.blog_dir / 'hugo.toml').exists():
            self.logger.error(f"Not a Hugo site (no config file): {blog_dir}")
            return
        
        # Check if Git is initialized
        if not (self.blog_dir / '.git').exists():
            self.logger.error(f"Hugo site not a Git repository: {blog_dir}")
            self.logger.info("Initialize with: cd grace-blog && git init")
            return
        
        self.posts_dir = self.blog_dir / 'content' / 'posts'
        self.posts_dir.mkdir(parents=True, exist_ok=True)
        
        self.enabled = True
        self.logger.info(f"✅ Hugo blogger initialized: {blog_dir}")
    
    def post_reflection(self, reflection_text: str, date_str: str, age_days: int, 
                       message_count: int) -> dict:
        """Post daily reflection to Hugo blog"""
        if not self.enabled:
            return {'success': False, 'error': 'Not enabled'}
        
        try:
            # Create filename
            filename = f'{date_str}-day-{age_days}.md'
            filepath = self.posts_dir / filename
            
            # Build Hugo front matter
            frontmatter = f"""---
title: "Day {age_days} Reflection"
date: {date_str}T23:59:59
draft: false
tags: ["daily-reflection", "ai-journal", "grace"]
categories: ["Daily Reflections"]
author: "Grace"
description: "Grace's daily reflection for day {age_days} - {message_count} conversations"
---

{reflection_text}

---

*This reflection was automatically generated from {message_count} conversations on day {age_days}.*
"""
            
            # Write file
            filepath.write_text(frontmatter)
            self.logger.info(f"✅ Created blog post: {filename}")
            self.logger.info(f"   Location: {filepath}")
            
            # Git commit and push (if auto-deploy enabled)
            if self.auto_deploy:
                deployed = self._git_deploy(filename, date_str)
                
                return {
                    'success': True,
                    'filename': filename,
                    'filepath': str(filepath),
                    'deployed': deployed,
                    'url': self._get_blog_url(filename)
                }
            else:
                return {
                    'success': True,
                    'filename': filename,
                    'filepath': str(filepath),
                    'deployed': False
                }
        
        except Exception as e:
            self.logger.error(f"Failed to post: {e}")
            import traceback
            traceback.print_exc()
            return {'success': False, 'error': str(e)}
    
    def _git_deploy(self, filename: str, date_str: str) -> bool:
        """Git commit and push"""
        try:
            self.logger.info("🔄 Running Git deployment...")
            
            # Stage file
            result = subprocess.run(
                ['git', 'add', f'content/posts/{filename}'],
                cwd=self.blog_dir,
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode != 0:
                self.logger.error(f"Git add failed: {result.stderr}")
                return False
            
            self.logger.info("   ✅ Git add successful")
            
            # Commit
            commit_message = f'Add test reflection for {date_str}'
            result = subprocess.run(
                ['git', 'commit', '-m', commit_message],
                cwd=self.blog_dir,
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode != 0:
                if "nothing to commit" in result.stdout or "nothing to commit" in result.stderr:
                    self.logger.info("   ℹ️  Nothing new to commit")
                    return True
                else:
                    self.logger.error(f"Git commit failed: {result.stderr}")
                    return False
            
            self.logger.info(f"   ✅ Git commit successful")
            
            # Push to GitHub
            self.logger.info("   🚀 Pushing to GitHub...")
            result = subprocess.run(
                ['git', 'push', HUGO_GIT_REMOTE, HUGO_GIT_BRANCH],
                cwd=self.blog_dir,
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                self.logger.error(f"Git push failed: {result.stderr}")
                self.logger.info("   Make sure remote is configured:")
                self.logger.info("   git remote add origin https://github.com/USERNAME/grace-blog.git")
                return False
            
            self.logger.info("   ✅ Pushed to GitHub!")
            self.logger.info("   ⏳ GitHub Actions will build and deploy (takes 1-2 minutes)")
            
            return True
        
        except Exception as e:
            self.logger.error(f"Git error: {e}")
            return False
    
    def _get_blog_url(self, filename: str) -> str:
        """Get blog URL"""
        slug = filename.replace('.md', '')
        
        try:
            result = subprocess.run(
                ['git', 'remote', 'get-url', HUGO_GIT_REMOTE],
                cwd=self.blog_dir,
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                remote_url = result.stdout.strip()
                
                if 'github.com' in remote_url:
                    if 'github.com/' in remote_url:
                        parts = remote_url.split('github.com/')[1].split('/')
                        username = parts[0]
                        repo = parts[1].replace('.git', '')
                    elif 'github.com:' in remote_url:
                        parts = remote_url.split('github.com:')[1].split('/')
                        username = parts[0]
                        repo = parts[1].replace('.git', '')
                    else:
                        username = 'USERNAME'
                        repo = 'grace-blog'
                    
                    return f'https://{username}.github.io/{repo}/posts/{slug}/'
        except:
            pass
        
        return f'https://USERNAME.github.io/grace-blog/posts/{slug}/'
    
    def run_tests(self) -> bool:
        """Run deployment tests"""
        self.logger.info("\n🔍 Running deployment tests...\n")
        
        checks_passed = 0
        total_checks = 5
        
        # Test 1: Hugo installed
        self.logger.info("1. Checking Hugo installation...")
        try:
            result = subprocess.run(['hugo', 'version'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                version = result.stdout.strip()
                self.logger.info(f"   ✅ {version}")
                checks_passed += 1
            else:
                self.logger.error("   ❌ Hugo not found")
                self.logger.info("   Install: sudo snap install hugo")
        except:
            self.logger.error("   ❌ Hugo not installed")
        
        # Test 2: Git configured
        self.logger.info("\n2. Checking Git repository...")
        try:
            result = subprocess.run(['git', 'status'], cwd=self.blog_dir, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.logger.info("   ✅ Git repository is valid")
                checks_passed += 1
            else:
                self.logger.error("   ❌ Git repository has issues")
        except:
            self.logger.error("   ❌ Git not configured")
        
        # Test 3: Remote configured
        self.logger.info("\n3. Checking GitHub remote...")
        try:
            result = subprocess.run(['git', 'remote', '-v'], cwd=self.blog_dir, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                remotes = result.stdout.strip()
                if 'github.com' in remotes:
                    self.logger.info("   ✅ GitHub remote configured")
                    for line in remotes.split('\n')[:2]:  # Show first 2 lines
                        self.logger.info(f"      {line}")
                    checks_passed += 1
                else:
                    self.logger.error("   ❌ No GitHub remote found")
                    self.logger.info("   Add with: git remote add origin https://github.com/USERNAME/grace-blog.git")
            else:
                self.logger.error("   ❌ Failed to check remotes")
        except:
            self.logger.error("   ❌ Remote check failed")
        
        # Test 4: Posts directory writable
        self.logger.info("\n4. Checking posts directory...")
        try:
            test_file = self.posts_dir / '.test'
            test_file.write_text('test')
            test_file.unlink()
            self.logger.info(f"   ✅ Posts directory writable: {self.posts_dir}")
            checks_passed += 1
        except:
            self.logger.error("   ❌ Cannot write to posts directory")
        
        # Test 5: Hugo config
        self.logger.info("\n5. Checking Hugo configuration...")
        try:
            config_file = self.blog_dir / 'hugo.toml' if (self.blog_dir / 'hugo.toml').exists() else self.blog_dir / 'config.toml'
            if config_file.exists():
                config = config_file.read_text()
                if 'baseURL' in config:
                    base_url = [line for line in config.split('\n') if 'baseURL' in line][0]
                    self.logger.info(f"   ✅ Hugo config found")
                    self.logger.info(f"      {base_url.strip()}")
                    checks_passed += 1
                else:
                    self.logger.warn("   ⚠️  baseURL not set in config")
                    checks_passed += 1
            else:
                self.logger.error("   ❌ Hugo config not found")
        except:
            self.logger.error("   ❌ Config check failed")
        
        # Summary
        self.logger.info("\n" + "="*50)
        self.logger.info(f"Test Results: {checks_passed}/{total_checks} checks passed")
        self.logger.info("="*50 + "\n")
        
        return checks_passed >= 4  # Need at least 4/5 to proceed


def main():
    """Run test blog posting"""
    
    print("="*60)
    print("🧪 Grace Hugo Blog Test")
    print("="*60)
    print()
    
    logger = SimpleLogger()
    
    # Check blog directory
    if not Path(HUGO_BLOG_DIR).exists():
        logger.error(f"Blog directory not found: {HUGO_BLOG_DIR}")
        logger.info("\nSetup instructions:")
        logger.info("1. Create Hugo site:")
        logger.info("   cd ~ && hugo new site grace-blog")
        logger.info("2. Add theme:")
        logger.info("   cd grace-blog")
        logger.info("   git init")
        logger.info("   git submodule add https://github.com/adityatelange/hugo-PaperMod themes/PaperMod")
        logger.info("   echo \"theme = 'PaperMod'\" >> hugo.toml")
        logger.info("3. Setup GitHub:")
        logger.info("   git remote add origin https://github.com/OppaAI/grace-blog.git")
        sys.exit(1)
    
    # Initialize blogger
    blogger = HugoBlogger(
        blog_dir=HUGO_BLOG_DIR,
        logger=logger,
        auto_deploy=True
    )
    
    if not blogger.enabled:
        logger.error("Hugo blogger failed to initialize")
        sys.exit(1)
    
    # Run tests
    if not blogger.run_tests():
        logger.error("Some tests failed. Fix issues before posting.")
        response = input("\nContinue anyway? (y/n): ")
        if response.lower() != 'y':
            sys.exit(1)
    
    # Create test reflection
    print()
    logger.info("📝 Creating test blog post...")
    print()
    
    today = datetime.now().date().isoformat()
    
    test_reflection = """🧪 This is a test reflection from Grace's blogging system!

I'm testing the automated Hugo + GitHub Pages integration. If you're reading this on your blog, it means everything is working perfectly! 🎉

**What this test proves:**
- ✅ Hugo markdown posts are created correctly
- ✅ Git commits work automatically  
- ✅ GitHub push succeeds
- ✅ GitHub Actions deployment is triggered
- ✅ Posts appear on the live site

This is just a test, but imagine this happening automatically every day with Grace's actual reflections. Pretty cool, right? 🚀

Tomorrow (or rather, in production), these will be real reflections generated from the day's conversations. For now, this test post confirms the plumbing works!"""
    
    # Post it
    result = blogger.post_reflection(
        reflection_text=test_reflection,
        date_str=today,
        age_days=1,  # Test day
        message_count=1  # Test count
    )
    
    # Show results
    print()
    print("="*60)
    if result['success']:
        logger.info("✅ TEST POST SUCCESSFUL!")
        print("="*60)
        print()
        logger.info(f"📄 File created: {result['filename']}")
        logger.info(f"📂 Location: {result['filepath']}")
        
        if result.get('deployed'):
            print()
            logger.info("🚀 DEPLOYED TO GITHUB!")
            logger.info(f"🌐 URL: {result['url']}")
            print()
            logger.info("⏳ Wait 1-2 minutes for GitHub Actions to build")
            logger.info("   Then check your blog!")
            print()
            logger.info("📊 Check deployment status:")
            logger.info("   https://github.com/OppaAI/grace-blog/actions")
        else:
            print()
            logger.warn("⚠️  Post created but not deployed (auto-deploy disabled)")
            logger.info("   Commit manually with:")
            logger.info(f"   cd {HUGO_BLOG_DIR}")
            logger.info("   git add .")
            logger.info("   git commit -m 'Test post'")
            logger.info("   git push")
    else:
        logger.error("❌ TEST POST FAILED")
        print("="*60)
        logger.error(f"Error: {result.get('error')}")
    
    print()
    print("="*60)
    print("🎉 Test complete!")
    print("="*60)


if __name__ == '__main__':
    main()