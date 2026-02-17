#!/usr/bin/env python3
"""
Nature Exploration Skills MCP Server (FIXED PROTOCOL)

Provides outdoor/nature exploration tools for Grace
"""

import asyncio
import sys
from datetime import datetime
from typing import Any
import requests
import math

# MCP Server using STDIO
from mcp.server.models import InitializationOptions
from mcp.server import NotificationOptions, Server
from mcp.server.stdio import stdio_server
from mcp import types

# Configuration
import os
DEFAULT_LAT = float(os.getenv('USER_LATITUDE', '49.2827'))
DEFAULT_LON = float(os.getenv('USER_LONGITUDE', '-123.1207'))
DEFAULT_LOCATION = os.getenv('USER_LOCATION', 'Vancouver, BC, Canada')

WEATHER_API = "https://api.open-meteo.com/v1/forecast"
AURORA_API = "https://services.swpc.noaa.gov/json/ovation_aurora_latest.json"
GEOCODING_API = "https://geocoding-api.open-meteo.com/v1/search"

# Helper functions
def calculate_moon_phase(date: datetime) -> dict:
    """Calculate moon phase"""
    year, month, day = date.year, date.month, date.day
    if month < 3:
        year -= 1
        month += 12
    
    a = year // 100
    b = a // 4
    c = 2 - a + b
    e = int(365.25 * (year + 4716))
    f = int(30.6001 * (month + 1))
    jd = c + day + e + f - 1524.5
    
    days_since_new = jd - 2451549.5
    new_moons = days_since_new / 29.53
    phase = (new_moons - int(new_moons))
    
    phases = [
        (0.0625, "New Moon 🌑"),
        (0.1875, "Waxing Crescent 🌒"),
        (0.3125, "First Quarter 🌓"),
        (0.4375, "Waxing Gibbous 🌔"),
        (0.5625, "Full Moon 🌕"),
        (0.6875, "Waning Gibbous 🌖"),
        (0.8125, "Last Quarter 🌗"),
        (1.0, "Waning Crescent 🌘")
    ]
    
    phase_name = "New Moon 🌑"
    for threshold, name in phases:
        if phase < threshold:
            phase_name = name
            break
    
    illumination = round(100 * (1 - abs(phase - 0.5) * 2), 1)
    
    return {
        "phase": phase_name,
        "illumination": illumination,
        "age_days": round(phase * 29.53, 1)
    }

def calculate_sun_times(lat: float, lon: float, date: datetime) -> dict:
    """Calculate sunrise/sunset (simplified)"""
    day_of_year = date.timetuple().tm_yday
    declination = -23.45 * math.cos(math.radians(360 / 365 * (day_of_year + 10)))
    
    lat_rad = math.radians(lat)
    dec_rad = math.radians(declination)
    cos_hour_angle = -math.tan(lat_rad) * math.tan(dec_rad)
    
    if cos_hour_angle > 1:
        return {"sunrise": "No sunrise (polar night)", "sunset": "No sunset", "day_length": "0:00"}
    elif cos_hour_angle < -1:
        return {"sunrise": "No sunrise (midnight sun)", "sunset": "No sunset", "day_length": "24:00"}
    
    hour_angle = math.degrees(math.acos(cos_hour_angle))
    solar_noon = 12 - (lon / 15)
    
    sunrise_hour = solar_noon - (hour_angle / 15)
    sunset_hour = solar_noon + (hour_angle / 15)
    
    def format_time(h):
        hour = int(h)
        minute = int((h - hour) * 60)
        return f"{hour:02d}:{minute:02d}"
    
    return {
        "sunrise": format_time(sunrise_hour),
        "sunset": format_time(sunset_hour),
        "day_length": format_time(2 * hour_angle / 15)
    }

async def get_coords(location: str):
    """Get coordinates for location"""
    try:
        response = requests.get(GEOCODING_API, params={"name": location, "count": 1}, timeout=10)
        data = response.json()
        results = data.get('results', [])
        if results:
            return results[0].get('latitude'), results[0].get('longitude')
    except:
        pass
    return DEFAULT_LAT, DEFAULT_LON

async def get_weather(lat: float, lon: float, days: int = 3):
    """Get weather forecast"""
    try:
        params = {
            "latitude": lat,
            "longitude": lon,
            "daily": "temperature_2m_max,temperature_2m_min,precipitation_sum",
            "current": "temperature_2m,relative_humidity_2m,wind_speed_10m,weather_code",
            "timezone": "auto",
            "forecast_days": days
        }
        response = requests.get(WEATHER_API, params=params, timeout=10)
        data = response.json()
        
        weather_codes = {
            0: "Clear ☀️", 1: "Mainly clear 🌤️", 2: "Partly cloudy ⛅",
            3: "Overcast ☁️", 45: "Foggy 🌫️", 61: "Rain 🌧️",
            71: "Snow 🌨️", 95: "Thunderstorm ⛈️"
        }
        
        current = data.get('current', {})
        code = current.get('weather_code', 0)
        
        return {
            "temperature": f"{current.get('temperature_2m', 'N/A')}°C",
            "humidity": f"{current.get('relative_humidity_2m', 'N/A')}%",
            "wind": f"{current.get('wind_speed_10m', 'N/A')} km/h",
            "conditions": weather_codes.get(code, "Unknown")
        }
    except Exception as e:
        return {"error": str(e)}

# Create server
server = Server("nature-skills")

@server.list_tools()
async def handle_list_tools() -> list[types.Tool]:
    """List all available tools"""
    return [
        types.Tool(
            name="get_weather",
            description="Get current weather and forecast for a location. Returns temperature, humidity, wind, precipitation, and multi-day forecast.",
            inputSchema={
                "type": "object",
                "properties": {
                    "location": {"type": "string", "description": "Location name (e.g., 'Vancouver, BC' or 'Banff, Alberta')"},
                    "days": {"type": "integer", "description": "Number of forecast days (1-7)", "default": 3}
                },
                "required": ["location"]
            }
        ),
        types.Tool(
            name="get_sun_times",
            description="Get sunrise, sunset, and day length for a location and date. Essential for planning outdoor activities.",
            inputSchema={
                "type": "object",
                "properties": {
                    "location": {"type": "string", "description": "Location name"},
                    "date": {"type": "string", "description": "Date in YYYY-MM-DD format (optional, defaults to today)"}
                },
                "required": ["location"]
            }
        ),
        types.Tool(
            name="get_moon_info",
            description="Get moon phase, illumination percentage, and age. Important for night navigation and wildlife observation.",
            inputSchema={
                "type": "object",
                "properties": {
                    "date": {"type": "string", "description": "Date in YYYY-MM-DD format (optional, defaults to today)"}
                }
            }
        ),
        types.Tool(
            name="get_aurora_forecast",
            description="Get aurora borealis (northern lights) forecast and visibility prediction. Use for planning aurora viewing trips.",
            inputSchema={
                "type": "object",
                "properties": {}
            }
        ),
        types.Tool(
            name="get_star_visibility",
            description="Calculate optimal stargazing conditions based on moon phase and location. Helps plan astronomy sessions.",
            inputSchema={
                "type": "object",
                "properties": {
                    "location": {"type": "string", "description": "Location name"},
                    "date": {"type": "string", "description": "Date in YYYY-MM-DD format (optional, defaults to today)"}
                },
                "required": ["location"]
            }
        ),
        types.Tool(
            name="get_wildlife_activity",
            description="Predict wildlife activity levels based on weather, time, and season. Helps plan wildlife observation.",
            inputSchema={
                "type": "object",
                "properties": {
                    "location": {"type": "string", "description": "Location name"}
                },
                "required": ["location"]
            }
        ),
        types.Tool(
            name="check_outdoor_conditions",
            description="Comprehensive outdoor conditions report including weather, sun times, moon phase, and safety recommendations. One-stop check for any outdoor activity.",
            inputSchema={
                "type": "object",
                "properties": {
                    "location": {"type": "string", "description": "Location name"},
                    "activity": {"type": "string", "description": "Planned activity (e.g., 'hiking', 'camping', 'photography', 'stargazing')"}
                },
                "required": ["location"]
            }
        ),
    ]

@server.call_tool()
async def handle_call_tool(
    name: str,
    arguments: dict[str, Any]
) -> list[types.TextContent]:
    """Handle tool calls"""
    
    try:
        if name == "get_weather":
            location = arguments.get("location", DEFAULT_LOCATION)
            days = min(arguments.get("days", 3), 7)
            
            lat, lon = await get_coords(location)
            weather = await get_weather(lat, lon, days)
            
            if "error" in weather:
                result = f"❌ Weather error: {weather['error']}"
            else:
                result = f"# Weather for {location}\n\n"
                result += f"🌡️ Temperature: {weather['temperature']}\n"
                result += f"💧 Humidity: {weather['humidity']}\n"
                result += f"💨 Wind: {weather['wind']}\n"
                result += f"☁️ Conditions: {weather['conditions']}\n"
            
            return [types.TextContent(type="text", text=result)]
        
        elif name == "get_sun_times":
            location = arguments.get("location", DEFAULT_LOCATION)
            date_str = arguments.get("date")
            date = datetime.fromisoformat(date_str) if date_str else datetime.now()
            
            lat, lon = await get_coords(location)
            sun_times = calculate_sun_times(lat, lon, date)
            
            result = f"# Sun Times for {location}\n\n"
            result += f"🌅 Sunrise: {sun_times['sunrise']}\n"
            result += f"🌇 Sunset: {sun_times['sunset']}\n"
            result += f"⏱️ Day Length: {sun_times['day_length']}\n"
            
            return [types.TextContent(type="text", text=result)]
        
        elif name == "get_moon_info":
            date_str = arguments.get("date")
            date = datetime.fromisoformat(date_str) if date_str else datetime.now()
            
            moon = calculate_moon_phase(date)
            
            result = f"# Moon Information\n\n"
            result += f"🌙 Phase: {moon['phase']}\n"
            result += f"💡 Illumination: {moon['illumination']}%\n"
            result += f"📅 Age: {moon['age_days']} days\n"
            
            return [types.TextContent(type="text", text=result)]
        
        elif name == "get_aurora_forecast":
            try:
                response = requests.get(AURORA_API, timeout=10)
                data = response.json()
                
                # Get max probability from coordinates
                coordinates = data.get('coordinates', [[]])
                aurora_data = coordinates[0] if coordinates else []
                
                max_prob = 0
                if aurora_data:
                    max_prob = max(max(row) for row in aurora_data if row) if aurora_data else 0
                
                if max_prob < 20:
                    visibility = "Very Low - No aurora visible"
                elif max_prob < 40:
                    visibility = "Low - Possible aurora at high latitudes"
                elif max_prob < 60:
                    visibility = "Moderate - Aurora visible in northern regions"
                elif max_prob < 80:
                    visibility = "High - Strong aurora activity expected"
                else:
                    visibility = "Very High - Excellent aurora viewing!"
                
                result = f"# Aurora Borealis Forecast 🌌\n\n"
                result += f"🌈 Probability: {max_prob}%\n"
                result += f"👁️ Visibility: {visibility}\n"
                result += f"⏰ Forecast Time: {data.get('Forecast Time', 'Unknown')}\n"
                result += f"\n💡 Best viewed in dark, clear skies away from light pollution\n"
                
            except Exception as e:
                result = f"❌ Aurora forecast error: {str(e)}"
            
            return [types.TextContent(type="text", text=result)]
        
        elif name == "get_star_visibility":
            location = arguments.get("location", DEFAULT_LOCATION)
            date_str = arguments.get("date")
            date = datetime.fromisoformat(date_str) if date_str else datetime.now()
            
            moon = calculate_moon_phase(date)
            illumination = moon['illumination']
            
            if illumination < 10:
                visibility = "Excellent ⭐⭐⭐⭐⭐"
                conditions = "New moon - perfect for stargazing!"
            elif illumination < 30:
                visibility = "Very Good ⭐⭐⭐⭐"
                conditions = "Crescent moon - great stargazing"
            elif illumination < 60:
                visibility = "Good ⭐⭐⭐"
                conditions = "Quarter moon - decent stargazing"
            elif illumination < 85:
                visibility = "Fair ⭐⭐"
                conditions = "Gibbous moon - some light pollution"
            else:
                visibility = "Poor ⭐"
                conditions = "Full moon - bright night sky"
            
            result = f"# Stargazing Conditions for {location}\n\n"
            result += f"⭐ Visibility: {visibility}\n"
            result += f"🌙 Conditions: {conditions}\n"
            result += f"🌕 Moon Phase: {moon['phase']}\n"
            result += f"💡 Illumination: {illumination}%\n"
            result += f"\n💡 Best viewing: After astronomical twilight, away from city lights\n"
            
            return [types.TextContent(type="text", text=result)]
        
        elif name == "get_wildlife_activity":
            location = arguments.get("location", DEFAULT_LOCATION)
            
            # Get weather for temperature
            lat, lon = await get_coords(location)
            weather = await get_weather(lat, lon, 1)
            
            temp = 15  # Default
            if "error" not in weather and weather.get('temperature'):
                try:
                    temp = float(weather['temperature'].replace('°C', ''))
                except:
                    pass
            
            # Determine time and season
            now = datetime.now()
            hour = now.hour
            
            if 5 <= hour < 7:
                time_of_day = "dawn"
            elif 19 <= hour < 21:
                time_of_day = "dusk"
            elif 21 <= hour or hour < 5:
                time_of_day = "night"
            else:
                time_of_day = "day"
            
            month = now.month
            if 3 <= month <= 5:
                season = "spring"
            elif 6 <= month <= 8:
                season = "summer"
            elif 9 <= month <= 11:
                season = "fall"
            else:
                season = "winter"
            
            # Activity level
            if 10 <= temp <= 25:
                activity = "High"
            elif temp < 0:
                activity = "Low"
            else:
                activity = "Moderate"
            
            recommendations = []
            
            # Time-based
            if time_of_day in ["dawn", "dusk"]:
                recommendations.append("🦌 Excellent time for deer and elk sightings")
                recommendations.append("🦅 Birds are most active")
            elif time_of_day == "night":
                recommendations.append("🦉 Good for nocturnal animals (owls, bats, raccoons)")
            
            # Season-based
            season_tips = {
                "spring": "🐻 Bears emerging from hibernation, bird migration",
                "summer": "🦋 Peak insect activity, young animals visible",
                "fall": "🦌 Rutting season for deer, animals preparing for winter",
                "winter": "🦅 Predators more visible, tracking easier in snow"
            }
            if season in season_tips:
                recommendations.append(season_tips[season])
            
            result = f"# Wildlife Activity Forecast\n\n"
            result += f"📊 Activity Level: {activity}\n\n"
            result += f"**Recommendations:**\n"
            for rec in recommendations:
                result += f"- {rec}\n"
            result += f"\n⚠️ Safety: Maintain safe distances, never feed wildlife\n"
            
            return [types.TextContent(type="text", text=result)]
        
        elif name == "check_outdoor_conditions":
            location = arguments.get("location", DEFAULT_LOCATION)
            activity = arguments.get("activity", "outdoor activity")
            
            lat, lon = await get_coords(location)
            weather = await get_weather(lat, lon, 1)
            sun_times = calculate_sun_times(lat, lon, datetime.now())
            moon = calculate_moon_phase(datetime.now())
            
            result = f"# Outdoor Conditions for {activity.title()}\n"
            result += f"📍 Location: {location}\n"
            result += f"📅 Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}\n\n"
            
            result += f"## Weather\n"
            if "error" not in weather:
                result += f"🌡️ {weather['temperature']}, {weather['conditions']}\n"
                result += f"💨 Wind: {weather['wind']}\n"
                result += f"💧 Humidity: {weather['humidity']}\n\n"
            
            result += f"## Daylight\n"
            result += f"🌅 Sunrise: {sun_times['sunrise']}\n"
            result += f"🌇 Sunset: {sun_times['sunset']}\n"
            result += f"⏱️ Day Length: {sun_times['day_length']}\n\n"
            
            result += f"## Moon\n"
            result += f"🌙 Phase: {moon['phase']}\n"
            result += f"💡 Illumination: {moon['illumination']}%\n\n"
            
            # Activity-specific tips
            result += f"## Recommendations for {activity.title()}\n"
            if activity.lower() in ["hiking", "trekking"]:
                result += "- Check trail conditions before departure\n"
                result += "- Bring headlamp if hiking near sunset\n"
                result += "- Pack layers for changing weather\n"
            elif activity.lower() == "camping":
                result += f"- Moon phase affects night visibility ({moon['illumination']}%)\n"
                result += "- Secure food from wildlife\n"
                result += "- Check weather forecast for precipitation\n"
            elif activity.lower() in ["photography", "stargazing"]:
                result += f"- Moon illumination: {moon['illumination']}%\n"
                result += "- Best shooting during golden hour (sunrise/sunset)\n"
                result += "- Clear skies needed for stars\n"
            else:
                result += "- Bring appropriate gear for conditions\n"
                result += "- Stay hydrated and protected from elements\n"
            
            return [types.TextContent(type="text", text=result)]
        
        else:
            return [types.TextContent(
                type="text",
                text=f"Unknown tool: {name}"
            )]
    
    except Exception as e:
        return [types.TextContent(
            type="text",
            text=f"Error: {str(e)}"
        )]

async def main():
    """Run the MCP server"""
    async with stdio_server() as (read_stream, write_stream):
        init_options = InitializationOptions(
            server_name="nature-skills",
            server_version="1.0.0",
            capabilities=server.get_capabilities(
                notification_options=NotificationOptions(),
                experimental_capabilities={},
            )
        )
        
        await server.run(
            read_stream,
            write_stream,
            init_options
        )

if __name__ == "__main__":
    asyncio.run(main())