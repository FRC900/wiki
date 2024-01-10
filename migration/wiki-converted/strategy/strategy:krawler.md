#  Instructions # 

##  Phone ## 
Turn on developer settings by going to: Settings>About phone>Build number and tapping on build number 10 times
Go to developer settings and select enable USB debugging

##  Installing ADB (Computer) ## 


<details>
<summary>
	Windows
</summary>
<p>

Install [Minimal ADB and Fastboot](https://www.androidfilehost.com/?fid=745425885120698566)

Run “Minimal ADB and Fastboot” from the startmenu
</p>
</details>

<details>
<summary>
Ubuntu
</summary>
<p>

```bash
sudo apt install android-tools-adb
```
</p>
</details>

##  Commands ## 
**Note that you should plug the MicroUSB cable into the tablet before plugging it into your computer**

<details>
<summary>
Windows
</summary>
<p>

##  For backing up: ## 
```cmd
adb backup -apk -f “C://krawlerbackup.ab” com.team2052.frckrawler
```

(use this to backup original appdata)

##  For Restore: ## 
```cmd
adb restore “C://krawlerbackup.ab”
```

(use this to restore appdata to new devices)
</p>
</details>

<details>
<summary>
Linux
</summary>
<p>

##  For backing up: ## 
```bash
adb backup -apk -f ~/krawlerbackup.ab com.team2052.frckrawler
```

(use this to backup original appdata)

##  For Restore: ## 
```bash
adb restore ~/krawlerbackup.ab
```

(use this to restore appdata to new devices)
</p>
</details>

---

[Google Doc Link](https://docs.google.com/a/wcpss.net/document/d/16wzQi7pLYccP4D4o-2Q8yiYmuZ-bXKT3LSN_Z7oo8ig/edit?usp=sharing)