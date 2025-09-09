# Autoware Multi-Year Roadmap

We have set the key objectives and multi-year goals we wish to achieve as AWF in alignment with our vision, mission and core values. Based on these key objectives and goals, we will then create an execution plan which scopes out how we will achieve our desired goals.

We have currently defined 8 Key Objectives covering three themes: Technology, Membership & Open Source Community, and Brand. For each Key Objective, we have defined goals over a three year period (Y1 - Y3).

## Key Objectives and Goals

### Technology

#### Objective 1: Incorporate cutting-edge technologies in Autoware’s software as part of an AI-first approach

- **Year 1:**
  - Introduce component-based AI stack for perception using key autonomous driving sensors - LIDAR, Vision and RADAR.
  - Develop an initial version of an AI-based planner to enable a full end-to-end AI-component-based autonomous driving stack - achieved through internal development and integration of technologies developed by member companies
- **Year 2:**
  - Introduce a first version of a single neural network end-to-end autonomous driving stack where sensor data is processed to directly compute safe driving trajectories
- **Year 3:**
  - Introduce a full hybrid AI stack with end-to-end autonomous driving as the primary mode and a component based AI-stack in parallel for additional safety and as a base technology layer.

#### Objective 2: Develop Autoware’s software towards production-ready solutions that can be more easily commercialized by members

- **Year 1:**
  - Create a first version of Production-Capable Autoware tightly integrated with OpenADKit for Low Speed Autonomy (warehouses, cargo, geo-fenced environments) and Privately Owned Vehicles (Highway Autonomous Pilot) at TRL-6.
- **Year 2:**
  - Continue development of Production-Capable Autoware for Low Speed Autonomy and Privately Owned Vehicles to achieve TRL-7 based on feedback from Autoware Members/Trial Partners.
  - Create a first version of Production-Capable Autoware tightly integrated with OpenADKit for Robobus (urban driving, fixed routes) and Off-Road Vehicles (Mining, Logistics, Off-road transport, etc) at TRL-6.
- **Year 3:**
  - Continue development of Production-Capable Autoware for Low Speed Autonomy and Privately Owned Vehicles to achieve TRL-8 based on feedback from Autoware Members/Trial Partners.
  - Continue development of Production-Capable Autoware for Robobus and Off-Road Vehicles to achieve TRL-7 based on feedback from Autoware Members/Trial Partners
  - Create a first version of Production-Capable Autoware tightly integrated with OpenADKit for Robotaxi at TRL-6.

#### Objective 3: Increase deployments of Autoware’s software to a greater number of vehicles, driving more miles autonomously

- **Year 1:**
  - Secure a pilot with at least one anchor company to trial Autoware’s Production-Capable stack at TRL-6 for Low Speed Autonomy and Privately Owned Vehicles on a single vehicle in real-world environments - potential anchor companies for Low Speed Autonomy include (Eve Autonomy, Whale Dynamic and Robeff), potential anchor companies for Privately Owned Vehicles include (AMD, DeepenAI, and AutoCore.ai).
- **Year 2:**
  - Secure a pilot with at least one anchor company to trial Autoware’s Production-Capable stack at TRL-7 for Low Speed Autonomy and Privately Owned Vehicles in a small-scale deployment across multiple vehicles in real-world environments.
  - Secure a pilot with at least one anchor company to trial Autoware’s Production-Capable stack at TRL-6 for Robobus and Off-Road Vehicles on a single vehicle in real-world environments - potential anchor company candidates for Robobus (Adastec), potential anchor company candidates for Off-Road Vehicles (DriveBlocks).
- **Year 3:**
  - Secure a pilot with at least one anchor company to trial Autoware’s Production-Capable stack at TRL-8 for Low Speed Autonomy and Privately Owned Vehicles across a larger scale deployment in a vehicle fleet in real-world environments.
  - Secure a pilot with at least one anchor company to trial Autoware’s Production-Capable stack at TRL-7 for Robobus and Off-Road Vehicles in a small-scale deployment across multiple vehicles in real-world environments.
  - Secure a pilot with at least one anchor company to trial Autoware’s Production-Capable stack at TRL-6 for Robotaxi on a single vehicle in real-world environments - potential anchor company candidates for Robotaxi include (TierIV)

### Membership & Open Source Community

#### Objective 4: Improve the ease with which new users can get started with Autoware and begin open-source contributions, and integration of Autoware technologies within their pre-existing software stack

- **Year 1:**
  - Create new refreshed documentation for Autoware, video tutorials and on-boarding guides to help users more easily understand how to get started with Autoware.
  - Measure new user satisfaction through surveys to assess ease of on-boarding process.
  - Introduce clear versioning/naming for Autoware software releases with LTS closely linked to documentation and tutorials.
- **Year 2:**
  - Create a ROS Index style list for all Autoware packages - searchable directory with details such as version, publishing date, maintainer, description, requirements etc.
  - Create a GUI based Autoware Configuration Dashboard tool to easily start, debug and manage Autoware’s autonomous driving stack.
- **Year 3:**
  - Create a first version of a proof-of-concept remote monitoring system for Autoware which allows logging of basic data such as GPS location of the vehicle, sensor log data, basic telemetry from GPS, live camera streams, and other simple information on an online dashboard.
  - Investigate methods to integrate V2X Messages and Infotainment related information into the Remote Monitoring system.

#### Objective 5: Grow Autoware’s open-source community, especially Premium Members and Individual Contributors

- **Year 1:**
  - Develop and implement a two-tiered marketing plan to on-board more Premium Members, and Individual contributors through tactics such as increased social media marketing (focus on video content - YouTube and LinkedIn as primary platforms), Press Releases (CISION etc.), content marketing (blogs and monthly newsletters - AWF Website, Medium, Substack etc.), SEO, Technology Influencer Collaborations (ride/demo in AWF powered car - could garner large number of views) and engagement on platforms with developer heavy communities (e.g. Reddit/Discord/HackerNews)
  - Add at least 12 New Premium Members
  - Add at least 25 new open-source contributors (not associated with Premium Members) actively participating (attending WG meetings and contributing code) in Autoware
  - Achieve over 15,000 Github stars in total across all AWF repositories.
- **Year 2:**
  - Continue execution of two-tiered marketing plan to on-board more Premium Members, and Individual contributors
  - Add at least 20 New Premium Members
  - Add at least 40 new open-source contributors (not associated with Premium Members) actively participating (attending WG meetings and contributing code) in Autoware
  - Achieve over 20,000 Github stars in total across all AWF repositories.
- **Year 3:**
  - Continue execution of two-tiered marketing plan to on-board more Premium Members, and Individual contributors
  - Add at least 30 New Premium Members
  - Add at least 60 new open-source contributors (not associated with Premium Members) actively participating (attending WG meetings and contributing code) in Autoware
  - Achieve over 25,000 Github stars in total across all AWF repositories.

#### Objective 6: Improve engagement with Autoware members and the community at large for a tight feedback loop which helps foster stronger collaboration and active participation by the community

- **Year 1:**
  - Assign a key individual for Premium Member engagement - who is responsible for holding regular touchpoint meetings with each Premium Member to help bolster their active participation, tracking w.r.t committed AWF plan and contributions within the community/usage of Autoware in their applications
  - Host another round of Autoware Technical Challenges with monetary prizes to encourage top-tier R&D submissions for Autoware’s most pressing technology challenges
  - Host the first Annual Autoware Awards Ceremony (similar to ROS Awards), recognizing and rewarding efforts by both individual open-source contributors and members companies - ‘Oscars for Autoware’ - for example: Individual contributors could be given certificates/monetary awards for authoring a certain number of commits, fixing a certain number of bugs, or being recognized for hard work by the WG chair and Member companies could be awarded financial prizes based on their contributions to Autoware on factors such as real-world deployments, community participation, open-source code contributions etc.
- **Year 2:**
  - Increase direct communication with Premium Members through hosting of Monthly Town Halls and other opportunities (e.g. Pitch Events/Company Showcases) for Premium Members to engage with the Foundation at large
  - Host Monthly events, hackathons and online lectures/sessions/show-and-tell/tutorials which are more friendly for individual developers to help grow open-source contributor base.
  - Continue hosting Autoware Technical Challenges and Annual Autoware Awards Ceremony
- **Year 3:**
  - Organize and hold the first Annual Autoware Conference for Member Companies and Open-Source Community at large, new potential member companies, other foundations, start-ups, individual developers, etc. over multiple days - covering showcases, pitch events, panel discussions, hackathons.
  - Continue hosting Monthly events for engagement with Premium Members and Open-Source contributors
  - Continue hosting Autoware Technical Challenges and Annual Autoware Awards Ceremony

#### Objective 7: Improve networking between members for sharing of business opportunities, enhancement of lead generation, and revenue growth

- **Year 1:**
  - Update and re-brand Autoware.io with latest information on companies and understand company requirements - e.g. hardware partner, demonstration partner, customer, etc.
  - Assign a dedicated team member responsible for organizing match-making sessions between companies based on their requirements
- **Year 2:**
  - Host monthly business development events allowing Autoware companies to pitch their products, open a call-for-participation, and raise their brand awareness - allow anyone to attend to maximise impact - e.g. DeepenAI/ASAM/PoV WG event recently held in the Bay Area.
- **Year 3:**
  - Host the first Autoware Accelerator/Grant Program inviting start-ups to partner with key Autoware members around specific business themes/projects with a cash-prize (similar in spirit to Autoware Challenges but more focused on business development).

### Brand

#### Objective 8: Improve Autoware’s brand positioning as a market leader in the autonomous driving space

- **Year 1:**
  - Get featured in at least one top publication focused on technology - Forbes, TechCrunch, WSJ, Sifted, Wired, Bloomberg etc. through Press Release timed with a key announcement (e.g. partnership, event, technology advancement etc.) and engagement with boutique PR firm/outreach to journalists.
  - Attend and present at (at least one) prestigious technology-first event (besides CES) e.g. WebSummit, MWC, SLUSH, SHIFT, Viva Technology, London TechWeek, etc.
- **Year 2:**
  - Create a video series highlighting well known Autoware member companies and how they are using Autoware and any completed demos (e.g. ARM, TIER IV, AMD, etc.) with interviews, graphics, high production quality and share across social media (organic and promoted) - aim to be picked up in press.
- **Year 3:**
  - Submit Autoware’s technology stack for various open autonomous driving benchmarks for Perception, Planning, End-to-End autonomy etc, aiming to achieve a pole position (1st - 3rd rank).
