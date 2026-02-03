// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as path from "path";
import { promises as fsPromises } from "fs";
import * as vscode from "vscode";

import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";
import * as vscode_utils from "../vscode-utils";

// Re-export common shell utilities
export { 
    detectUserShell, 
    getSetupScriptExtension, 
    ShellInfo 
} from "@ranchhandrobotics/rde-common";

import {
    detectUserShell,
    sourceSetupFile as commonSourceSetupFile,
    SourceSetupOptions
} from "@ranchhandrobotics/rde-common";

/**
 * Executes a setup file and returns the resulting env.
 * This wraps the common sourceSetupFile with ROS-specific logging.
 */
export function sourceSetupFile(filename: string, env?: any): Promise<any> {
    const config = vscode_utils.getExtensionConfiguration();
    const pixiRoot = config.get("pixiRoot", "c:\\pixi_ws");
    
    const options: SourceSetupOptions = {
        cwd: vscode.workspace.rootPath,
        pixiRoot,
        onOutput: (message: string) => {
            extension.outputChannel.appendLine(message);
        }
    };
    
    return commonSourceSetupFile(filename, env, options);
}

/**
 * Executes a shell command and returns the resulting env.
 * This allows ROS setup to be provided as a command string.
 */
export function sourceSetupCommand(command: string, env?: any): Promise<any> {
    const shellInfo = detectUserShell();
    const spawnOptions: child_process.SpawnOptionsWithoutStdio = {
        cwd: vscode.workspace.rootPath,
        env: env || process.env,
        windowsHide: true,
    };

    const isWindows = process.platform === "win32";
    const executable = isWindows ? "cmd.exe" : shellInfo.executable;
    const args = isWindows
        ? ["/d", "/s", "/c", `${command} && set`]
        : ["--login", "-c", `${command}; env -0`];

    return new Promise((resolve, reject) => {
        const proc = child_process.spawn(executable, args, spawnOptions);
        const stdoutChunks: Buffer[] = [];
        const stderrChunks: Buffer[] = [];

        proc.stdout.on("data", (data: Buffer) => stdoutChunks.push(data));
        proc.stderr.on("data", (data: Buffer) => stderrChunks.push(data));

        proc.on("error", (error) => {
            reject(error);
        });

        proc.on("close", (code) => {
            const stdoutText = Buffer.concat(stdoutChunks).toString("utf8");
            const stderrText = Buffer.concat(stderrChunks).toString("utf8").trim();

            if (stderrText) {
                extension.outputChannel.appendLine(stderrText);
            }

            if (code !== 0) {
                reject(new Error(`Failed to source ROS setup command (exit code ${code}).`));
                return;
            }

            const parsedEnv = isWindows ? parseWindowsSetOutput(stdoutText) : parseNullSeparatedEnv(stdoutText);
            resolve(parsedEnv);
        });
    });
}

function parseNullSeparatedEnv(stdoutText: string): any {
    const parsedEnv: any = {};
    for (const entry of stdoutText.split("\0")) {
        if (!entry) {
            continue;
        }

        const separatorIndex = entry.indexOf("=");
        if (separatorIndex <= 0) {
            continue;
        }

        const key = entry.substring(0, separatorIndex);
        const value = entry.substring(separatorIndex + 1);
        parsedEnv[key] = value;
    }
    return parsedEnv;
}

function parseWindowsSetOutput(stdoutText: string): any {
    const parsedEnv: any = {};
    for (const line of stdoutText.split(/\r?\n/g)) {
        if (!line) {
            continue;
        }

        const separatorIndex = line.indexOf("=");
        if (separatorIndex <= 0) {
            continue;
        }

        const key = line.substring(0, separatorIndex);
        const value = line.substring(separatorIndex + 1);
        parsedEnv[key] = value;
    }
    return parsedEnv;
}

export function xacro(filename: string): Promise<any> {
    return new Promise((resolve, reject) => {
        let processOptions = {
            cwd: vscode.workspace.rootPath,
            env: extension.env,
            windowsHide: false,
        };

        let xacroCommand: string;
        if (process.platform === "win32") {
            xacroCommand = `cmd /c "xacro "${filename}""`;
        } else {
            const shellInfo = detectUserShell();
            xacroCommand = `${shellInfo.executable} --login -c "xacro '${filename}'"`;
        }

        child_process.exec(xacroCommand, processOptions, (error, stdout, _stderr) => {
            if (!error) {
                resolve(stdout);
            } else {
                reject(error);
            }
        });
    });
}

/**
 * Gets the names of installed distros.
 */
export function getDistros(): Promise<string[]> {
    try {
        return fsPromises.readdir("/opt/ros");
    } catch (error) {
        return Promise.resolve([]);
    }
}

/**
 * Creates and shows a ROS-sourced terminal.
 */
export function createTerminal(context: vscode.ExtensionContext): vscode.Terminal {
    const reporter = telemetry.getReporter();
    reporter.sendTelemetryCommand(extension.Commands.CreateTerminal);

    const terminal = vscode.window.createTerminal({ name: 'ros2', env: extension.env })
    terminal.show();

    return terminal;
}
