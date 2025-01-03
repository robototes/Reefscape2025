# ##########################################################################
#
# Gradle startup script for Windows Powershell.
#
# Ported from gradlew.bat for use on Windows where .bat files are blocked.
#
# Can be use from Powershell Terminal in Visual Studio Code.
#  Example usage to build :
#    .\gradlew.ps1 build
#
# TODO : Does it help to install powershell extensions plugin in VSCode??
#
# Original gradlew.bat file is licensed under the Apache license.
################################################################################
#
# Copyright 2015 the original author or authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ##########################################################################

$APP_HOME = Split-Path $MyInvocation.MyCommand.Path
$APP_NAME = 'gradlew'

# Add default JVM options here. You can also use JAVA_OPTS and GRADLE_OPTS to pass JVM options to this script.
$DEFAULT_JVM_OPTS = '-Xms64m -Xmx64m'

# Find java.exe
if (! $env:JAVA_HOME) {
	$JAVA_EXE = 'java.exe'
	$p = '$JAVA_EXE -version >NUL 2>&1'
	if ($p.ExitCode) {
		Write-Output ""
		Write-Output "ERROR: JAVA_HOME is not set and no 'java' command could be found in your PATH."
		Write-Output ""
		Write-Output "Please set the JAVA_HOME variable in your environment to match the"
		Write-Output "location of your Java installation."
		exit 1
	}
} else {
	$JAVA_EXE = "$env:JAVA_HOME/bin/java.exe"
	if (-not (Test-Path -Path $JAVA_EXE -PathType Leaf)) {
		Write-Output ""
		Write-Output "ERROR: JAVA_HOME is set to an invalid directory: $env:JAVA_HOME"
		Write-Output ""
		Write-Output "Please set the JAVA_HOME variable in your environment to match the"
		Write-Output "location of your Java installation."
		exit 1
	}
}

# Include command line arguments
$CMD_LINE_ARGS = $args -join " "

# Setup the CLASSPATH
$CLASSPATH = "$APP_HOME\gradle\wrapper\gradle-wrapper.jar"

# Setup parameters
$params = "$DEFAULT_JVM_OPTS $env:JAVA_OPTS $env:GRADLE_OPTS -Dorg.gradle.appname=$APP_NAME -classpath $CLASSPATH org.gradle.wrapper.GradleWrapperMain $CMD_LINE_ARGS"
# Execute Gradle
$g = Start-Process -Wait -PassThru -NoNewWindow -FilePath $JAVA_EXE -ArgumentList $params
exit $g.ExitCode
