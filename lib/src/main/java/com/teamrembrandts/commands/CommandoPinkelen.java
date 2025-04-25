/*
 * Copyright (c) 2024-2025 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package com.teamrembrandts.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * 🥚🐇 - Dutch version of Simon Says.
 *
 * <p>'Commando pinkelen' is a concentration-oriented game that can be played by both children and adults. The number of
 * participants is unlimited.
 *
 * <p>In the simplest version, the game leader has four 'commands' at their disposal: the commands 'pinkelen', 'hollow',
 * 'round' and 'flat'. When the game leader gives the 'command pinkelen', the players drum with their index fingers on
 * the edge of the table; for the 'command hollow', the participants place their hand with the back downwards on the
 * table; for the 'command round', the hands are placed with the fingertips on the table; for the 'command flat', the
 * players place their hands flat on the table, with the palms downwards. However, if the game leader omits the word
 * 'command' in their command, the player is expected to ignore the command. Anyone who makes a mistake in this is out
 * and cannot participate any further.
 *
 * @author Nigel
 * @since 2025
 * @see <a href="https://nl.wikipedia.org/wiki/Commando_pinkelen">Commando pinkelen on Wikipedia</a>
 */
public class CommandoPinkelen extends Command {

    /** This class is meant as an Easter egg and shouldn't be instantiated. */
    private CommandoPinkelen() {}
}
