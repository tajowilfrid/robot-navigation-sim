package minisim.sim;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.GraphicsEnvironment;

import javax.swing.JFrame;
import javax.swing.JPanel;

import minisim.sim.Environment.Field;

public class MapGui extends JFrame 
{
	private Environment environment;
    private MapPanel mapPanel;

    public MapGui(Environment environment) 
    {
    	this.environment = environment;

        setTitle("Map Gui");
        setSize(400, 400);  
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);

        this.mapPanel = new MapPanel();
        getContentPane().add(mapPanel);  
    }

    private class MapPanel extends JPanel 
    {
        @Override
        protected void paintComponent(Graphics g) 
        {
            super.paintComponent(g);  
            
            Field[][] map = environment.getEnvironment();
            
            int cellSize = Math.min(getWidth() / map[0].length, getHeight() / map.length);
            
            for (int y = 0; y < map.length; y++) 
            {
                for (int x = 0; x < map[0].length; x++) 
                {
                	Color color;
                	String symbol = "";
                    
                    switch (map[y][x]) 
                    {
                        case EMPTY:
                            color = Color.WHITE;
                            symbol = "";
                            break;
                        case OBSTACLE:
                            color = Color.BLACK;
                            symbol = "O";
                            break;
                        case CHARGER:
                            color = Color.YELLOW;
                            symbol = "C";
                            break;
                        case LAVA:
                            color = Color.RED;
                            symbol = "L";
                            break;
                        case START:
                            color = Color.BLUE;
                            symbol = "S";
                            break;
                        case TARGET:
                            color = Color.GREEN;
                            symbol = "T";
                            break;
                        default:
                            color = Color.GRAY;
                    }

                    g.setColor(color);
                    g.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);

                    g.setColor(Color.BLACK); 
                    
                    int textWidth = g.getFontMetrics().stringWidth(symbol);
                    int textHeight = g.getFontMetrics().getAscent();
                    g.drawString(symbol, x * cellSize + (cellSize - textWidth) / 2, 
                    	y * cellSize + (cellSize + textHeight) / 2);

                    for (Robot r : environment.getRobots().values()) 
                    {
                        int robotX = r.getX();
                        int robotY = r.getY();

                        g.setColor(Color.BLUE);
                        g.fillOval(robotX * cellSize + 5, robotY * cellSize + 5, cellSize - 10, cellSize - 10);

                        g.setColor(Color.WHITE);
                        g.setFont(new Font("Arial", Font.BOLD, cellSize / 3)); 
                        String energyText = String.valueOf(r.getEnergy());
                        textWidth = g.getFontMetrics().stringWidth(energyText);
                        textHeight = g.getFontMetrics().getHeight();
                        g.drawString(energyText, robotX * cellSize + (cellSize - textWidth) / 2, 
                                               robotY * cellSize + (cellSize + textHeight / 4) / 2);
                    }

                    g.setColor(Color.GRAY);
                    g.drawRect(x * cellSize, y * cellSize, cellSize, cellSize);
                }
            }
        }
    }
}